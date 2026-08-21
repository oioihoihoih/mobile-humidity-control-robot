param(
    [string]$Port = $env:SERIAL_PORT
)

if ([string]::IsNullOrWhiteSpace($env:CONTROL_API_TOKEN)) {
    throw "Set CONTROL_API_TOKEN before exposing the server to the LAN."
}
if ([string]::IsNullOrWhiteSpace($Port)) {
    throw "Pass -Port <serial-port> or set SERIAL_PORT."
}

$env:SERIAL_ENABLED = "1"
$env:SERIAL_PORT = $Port
$env:ROBOT_BIND_HOST = "0.0.0.0"
$projectServer = Join-Path $PSScriptRoot "server.py"
& python $projectServer
