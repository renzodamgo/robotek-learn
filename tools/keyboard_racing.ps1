# Drive the real car like a racing game from a Windows PC: a small window, arrows / W A S D held and combined.
#
#   powershell -ExecutionPolicy Bypass -File tools\keyboard_racing.ps1 [-Speed 0.4] [-Steer 15] [-PiIp 192.168.8.153]
#
# Run it from a terminal where ROS 2 Jazzy is already loaded, for example:
#   - RoboStack with pixi: open your RoboStack folder and run `pixi shell` (or `pixi run powershell`);
#   - the official Windows binaries: `call C:\dev\ros2_jazzy\local_setup.bat` in cmd, then `powershell`.
# On the car first: ros2 launch robotek_course car.launch.py teleop:=true [motor_max_rps:=1.3]
#
# Same network as the car: Cyclone DDS, ROS_DOMAIN_ID 1, discovery in the subnet and the Pi as a static peer. The PC
# must be on the car's Wi-Fi, and Windows Defender Firewall must let Python receive UDP (it asks the first time:
# allow it on private networks). With several network cards, set CYCLONEDDS_URI to pick the Wi-Fi one, e.g.
#   $env:CYCLONEDDS_URI = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="Wi-Fi"/></Interfaces></General></Domain></CycloneDDS>'
param(
    [double]$Speed = 0.2,
    [double]$Steer = 15,
    [string]$PiIp = $(if ($env:PI_IP) { $env:PI_IP } else { '192.168.8.153' })
)

if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    Write-Error 'ROS 2 is not loaded in this terminal (no ros2 command): load your Jazzy environment first.'
    exit 1
}
$env:RMW_IMPLEMENTATION = 'rmw_cyclonedds_cpp'
$env:ROS_DOMAIN_ID = $(if ($env:REAL_CAR_DOMAIN_ID) { $env:REAL_CAR_DOMAIN_ID } else { '1' })
$env:ROS_AUTOMATIC_DISCOVERY_RANGE = 'SUBNET'
if (-not $env:ROS_STATIC_PEERS) { $env:ROS_STATIC_PEERS = $PiIp }

Write-Host "keyboard_racing: dominio $env:ROS_DOMAIN_ID, coche $PiIp, velocidad $Speed m/s, giro $Steer grados"
if (-not (Test-Connection -ComputerName $PiIp -Count 1 -Quiet)) {
    Write-Warning "No responde $PiIp (ping): ¿estás en la Wi-Fi del coche y está encendido?"
}
ros2 daemon stop *> $null
$script = Join-Path $PSScriptRoot 'keyboard_racing.py'
python $script $Speed $Steer
