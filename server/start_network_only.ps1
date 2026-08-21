if ([string]::IsNullOrWhiteSpace($env:CONTROL_API_TOKEN)) {
    throw "Set CONTROL_API_TOKEN before exposing the server to the LAN."
}

$env:SERIAL_ENABLED = "0"
$env:ROBOT_BIND_HOST = "0.0.0.0"
$projectServer = Join-Path $PSScriptRoot "server.py"
& python $projectServer
