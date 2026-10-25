"""Rule judge and standard validation for recorded simulation rounds.

``judge_open`` applies the WRO 2026 Open Challenge scoring rules to saved telemetry
(ground-truth pose used only for auditing). ``standard_validation`` builds the same
validation block for Open and Obstacle rounds.

Two different questions are kept apart:

* ``rule_judge``: what the rules would award (sections 1.1, laps 1.2, stop 1.3, and
  the round-end conditions of 9.24). Still a simulation, not an official score.
* ``validation``: our internal acceptance (rule maximum, no contact, margins, stop
  observed, steering limit). Thresholds are explicit and versioned.
"""
import json
import math

from . import contact_log
from .footprint import collision_polygons

JUDGE_VERSION = 2
VALIDATION_VERSION = 1
# Internal thresholds (not rules).
MIN_CLEARANCE_M = 0.01      # sampled clearance treated as "no near-contact"
GEOMETRIC_CONTACT_M = 0.008  # footprint approximation touches at about 6 mm
MARGIN_COMFORT_M = 0.05
MARGIN_TIGHT_M = 0.02
STEER_LIMIT_DEG = 25.5       # 25 deg plus measurement tolerance
STOP_RULE_S = 15.0           # 9.24.2 note 2
STOP_OBSERVED_S = 30.0       # Appendix A 2 / AGENTS.md
STATIONARY_M = 0.002
ROUND_LIMIT_S = 180.0


def mat_section(x, y):
    """Fixed mat sections, CCW order from the bottom straight: 0 bottom, 1 SE corner, ..."""
    if y < -.5:
        return 7 if x < -.5 else (1 if x > .5 else 0)
    if y > .5:
        return 5 if x < -.5 else (3 if x > .5 else 4)
    return 6 if x < 0 else 2


def footprint_section(vertices):
    sections = {mat_section(x, y) for x, y in vertices}
    return sections.pop() if len(sections) == 1 else None


def wall_sensor_present(directory):
    """Static walls only publish on contact: an empty log means 'no contact' only if
    the round's world snapshot carried the wall contact sensor."""
    for name in ('source/world.sdf', 'world.sdf'):
        path = directory/name
        if path.exists():
            return 'name="wall_contact"' in path.read_text()
    return False


def load_contacts(directory):
    path = directory/'contacts.jsonl'
    if not path.exists() or not wall_sensor_present(directory):
        return None
    return contact_log.read(path)


def wall_kind(name):
    if 'wall_ext' in name:
        return 'outer_wall'
    if 'wall_int' in name:
        return 'inner_wall'
    if 'sign_' in name:
        return 'pillar'
    if 'parking' in name or 'magenta' in name:
        return 'parking_limit'
    return 'other'


def judge_open(directory, manifest, rows, events):
    """Rule-based Open score from footprint section exits, laps, stop and round end."""
    sign = 1 if manifest.get('direction', 'ccw') == 'ccw' else -1
    start = next((e for e in events if e['event'] == 'start'), None)
    valid = [r for r in rows if r.get('x_gt') not in (None, '', 'nan')
             and math.isfinite(float(r['x_gt'])) and r.get('steer_actual') not in (None, '')]
    notes, faults = [], []
    result = dict(version=JUDGE_VERSION, challenge='open', max_points=30, points=0,
                  breakdown={'1.1_sections': 0, '1.2_laps': 0, '1.3_stop_in_finish': 0},
                  faults=faults, notes=notes, round_end=None, time_from_start_sim_seconds=None,
                  time_from_start_wall_seconds=None, lap_splits=[], contact_evaluated=False)
    if not valid or start is None:
        notes.append('No start event or no ground-truth samples: not judged.')
        return result
    t0, w0 = float(start['sim_time']), float(start['wall_time'])
    current = None
    exits = 0
    reverse = 0
    end = None
    laps = []
    stop_rows = []
    for r in valid:
        t = float(r['sim_time'])
        if t < t0:
            continue
        if t-t0 > ROUND_LIMIT_S:
            end = dict(reason='timer_expired', sim_time=t0+ROUND_LIMIT_S, wall_time=float(r['wall_time']))
            break
        vertices = sum(collision_polygons(r).values(), [])
        section = footprint_section(vertices)
        if current is None:
            current = section
            continue
        if section is None or section == current:
            pass
        elif section == (current+sign) % 8:
            if reverse:
                reverse -= 1  # back over a boundary already crossed forward: no new point
            else:
                exits += 1
                if exits % 8 == 0 and exits <= 24:
                    laps.append(dict(lap=exits//8, sim_time=t, wall_time=float(r['wall_time'])))
                if exits == 25:
                    # 9.24.3: after three laps it left the finish section completely.
                    end = dict(reason='passed_finish_section', sim_time=t, wall_time=float(r['wall_time']))
                    break
            current = section
        elif section == (current-sign) % 8:
            reverse += 1
            current = section
            if reverse >= 2:
                end = dict(reason='crossed_two_boundaries_backwards', sim_time=t, wall_time=float(r['wall_time']))
                faults.append(dict(rule='9.24.3/A4', fault='opposite_direction_two_boundaries', sim_time=t))
                break
        if exits >= 24:
            stop_rows.append(r)
    # 1.3: after three laps, a stop with the whole footprint inside the finish section.
    finish = 0
    if exits >= 24 and end is None:
        stop_start = None
        for r in stop_rows:
            if r.get('state') == 'DONE':
                stop_start = r
                break
        if stop_start is None:
            notes.append('Three laps but no DONE stop recorded.')
        else:
            ts = float(stop_start['sim_time'])
            held = [r for r in valid if float(r['sim_time']) >= ts+.5]
            if held:
                x0, y0 = float(held[0]['x_gt']), float(held[0]['y_gt'])
                drift = max(math.hypot(float(r['x_gt'])-x0, float(r['y_gt'])-y0) for r in held)
                hold = float(held[-1]['sim_time'])-float(held[0]['sim_time'])
                inside = footprint_section(sum(collision_polygons(held[0]).values(), [])) == current \
                    and current == _start_section(valid, t0)
                # Older runs ended telemetry soon after DONE and observed the stop
                # separately (stop-observation.json): use that observation.
                observed = directory/'stop-observation.json'
                if observed.exists():
                    obs = json.loads(observed.read_text())
                    if obs.get('sim_seconds') is not None and obs['sim_seconds'] > hold:
                        hold = float(obs['sim_seconds'])
                        drift = max(drift, float(obs.get('max_displacement_m') or 0.))
                        notes.append('Stop hold taken from stop-observation.json.')
                result.update(stop_hold_sim_seconds=hold, stop_drift_m=drift, stop_inside_finish_section=inside)
                if inside and drift <= STATIONARY_M and hold >= STOP_RULE_S:
                    finish = 3
                else:
                    notes.append('Stop not counted: inside=%s, hold=%.1f s, drift=%.4f m.' % (inside, hold, drift))
            done = next((e for e in events if e['event'] == 'done'), None)
            # The judge stops the clock at the stop; prefer the exact DONE event time.
            if done and abs(float(done['sim_time'])-ts) < 1.:
                end = dict(reason='stopped_after_three_laps', sim_time=float(done['sim_time']), wall_time=float(done['wall_time']))
            else:
                end = dict(reason='stopped_after_three_laps', sim_time=ts, wall_time=float(stop_start['wall_time']))
    if end is None and valid:
        last = valid[-1]
        end = dict(reason='end_of_recording', sim_time=float(last['sim_time']), wall_time=float(last['wall_time']))
    # 1.1: one point per section driven out of in the round direction (max 24); the
    # finish section and the one after it do not count (covered by the 24 cap).
    sections = min(24, exits)
    result['breakdown'] = {'1.1_sections': sections, '1.2_laps': len(laps), '1.3_stop_in_finish': finish}
    result['points'] = sections+len(laps)+finish
    result['round_end'] = end
    result['time_from_start_sim_seconds'] = end['sim_time']-t0
    result['time_from_start_wall_seconds'] = end['wall_time']-w0
    previous = t0
    for lap in laps:
        result['lap_splits'].append(dict(lap=lap['lap'], sim_seconds=lap['sim_time']-previous,
                                         cumulative_sim_seconds=lap['sim_time']-t0))
        previous = lap['sim_time']
    # Walls: rule 9.18 forbids touching the outer wall in the Open; the consequence is
    # not specified, so it is flagged and not deducted.
    contacts = load_contacts(directory)
    end_time = end['sim_time']
    if contacts is not None:
        result['contact_evaluated'] = True
        kinds = {}
        for c in contacts:
            if c['sim_time'] > end_time:
                continue
            kind = wall_kind(c.get('other', '') or c.get('pillar', '') or '')
            kinds.setdefault(kind, c['sim_time'])
        result['contacts_first_sim_time'] = kinds
        if 'outer_wall' in kinds:
            faults.append(dict(rule='9.18', fault='outer_wall_contact', sim_time=kinds['outer_wall'],
                               consequence='not specified by the rules; flagged, not deducted'))
    else:
        outer = _min_clearance(valid, manifest, t0, end_time)
        result['min_outer_clearance_geometric_m'] = outer
        if outer is not None and outer <= GEOMETRIC_CONTACT_M:
            faults.append(dict(rule='9.18', fault='outer_wall_contact_probable_geometric',
                               consequence='not specified by the rules; flagged, not deducted'))
        notes.append('No contact sensor data: outer-wall contact inferred from sampled geometry.')
    notes.append('Wall movement cannot occur in simulation (walls are static).')
    return result


def _start_section(valid, t0):
    for r in valid:
        if float(r['sim_time']) >= t0:
            return footprint_section(sum(collision_polygons(r).values(), []))
    return None


def _min_clearance(valid, manifest, t0, t1):
    outer = (manifest.get('field_geometry') or {}).get('outer')
    if not outer:
        return None
    best = math.inf
    for r in valid:
        t = float(r['sim_time'])
        if t < t0 or t > t1:
            continue
        for x, y in sum(collision_polygons(r).values(), []):
            best = min(best, x-outer['xmin'], outer['xmax']-x, y-outer['ymin'], outer['ymax']-y)
    return best if math.isfinite(best) else None


def margin_class(clearance):
    if clearance is None:
        return 'unknown'
    if clearance >= MARGIN_COMFORT_M:
        return 'comfortable'
    if clearance >= MARGIN_TIGHT_M:
        return 'tight'
    return 'risky'


def _check(value, threshold, passed, note=None):
    out = dict(value=value, threshold=threshold, passed=bool(passed))
    if note:
        out['note'] = note
    return out


def standard_validation(judge, clearances, contacts_evaluated, contacts_any, stop_hold, steer_max_deg):
    """Same acceptance for every challenge.

    clearances: {'outer_wall': m, 'inner_wall': m, 'pillar': m} (sampled, None if n/a).
    """
    checks = {}
    checks['rule_maximum'] = _check(judge['points'], judge['max_points'], judge['points'] >= judge['max_points'])
    checks['no_rule_faults'] = _check(len(judge['faults']), 0, not judge['faults'])
    checks['stop_observed'] = _check(stop_hold, STOP_OBSERVED_S, stop_hold is not None and stop_hold >= STOP_OBSERVED_S)
    if contacts_evaluated:
        checks['no_contacts'] = _check(bool(contacts_any), False, not contacts_any, 'Gazebo contact sensors')
    for name, value in clearances.items():
        if value is None:
            continue
        checks['min_%s_clearance_m' % name] = _check(value, MIN_CLEARANCE_M, value >= MIN_CLEARANCE_M,
                                                      'sampled at telemetry rate')
    checks['steering_limit'] = _check(steer_max_deg, STEER_LIMIT_DEG,
                                      steer_max_deg is not None and steer_max_deg <= STEER_LIMIT_DEG)
    known = [v for v in clearances.values() if v is not None]
    return dict(version=VALIDATION_VERSION, passed=all(c['passed'] for c in checks.values()),
                checks=checks, failed=[k for k, c in checks.items() if not c['passed']],
                min_clearance_m=min(known) if known else None,
                margin_class=margin_class(min(known) if known else None),
                thresholds=dict(min_clearance_m=MIN_CLEARANCE_M, comfortable_m=MARGIN_COMFORT_M,
                                tight_m=MARGIN_TIGHT_M, stop_observed_s=STOP_OBSERVED_S,
                                steer_limit_deg=STEER_LIMIT_DEG))
