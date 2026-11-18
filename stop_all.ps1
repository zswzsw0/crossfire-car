# Stop the run only: kill beaconRace on the car and let the camera preview take over.
# The VM viewer window is NEVER touched unless you pass -CloseViewer.
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\stop_all.ps1
#   powershell ... -File .\stop_all.ps1 -CloseViewer      (also kill the VM viewer)
#
# ASCII-only on purpose (PowerShell 5.1 reads a BOM-less .ps1 as ANSI).
param(
    [switch]$CloseViewer,
    [string]$BoardHost = 'root@192.168.1.3',
    [string]$VmHost = 'mark@192.168.52.133'
)
$ErrorActionPreference = 'Stop'
$SSH = Join-Path $env:WINDIR 'System32\OpenSSH\ssh.exe'

function Say([string]$t) { Write-Output ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $t) }

Say 'stopping beaconRace on the car (camera preview will take over, viewer stays up) ...'
$out = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $BoardHost 'sh /home/root/br_ctl.sh stop' 2>&1
Write-Output ($out | Out-String).Trim()

if ($CloseViewer) {
    Say 'closing the VM viewer (-CloseViewer) ...'
    $out = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $VmHost "pkill -f '^python3 /home/mark/gtk_viewer.py'" 2>&1
    Say 'viewer stopped'
} else {
    Say 'VM viewer left running'
}
Say 'done'
