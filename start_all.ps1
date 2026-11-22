# Start beaconRace on the board AND open the PC viewer -- one command.
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\start_all.ps1
#   powershell ... -File .\start_all.ps1 -Mode drive      (car really moves!)
#   powershell ... -File .\start_all.ps1 -Status          (only report state)
# ASCII-only on purpose: Windows PowerShell 5.1 reads .ps1 as ANSI without a BOM.
param(
    [ValidateSet('no-drive','drive')][string]$Mode = 'no-drive',
    [switch]$Status
)
$Board  = "192.168.1.3"
$Port   = 9999
$Here   = Split-Path -Parent $MyInvocation.MyCommand.Path
$Viewer = Join-Path $Here "viewer.ps1"

function Say([string]$t) { Write-Output ("[{0}] {1}" -f (Get-Date -Format "HH:mm:ss"), $t) }
function Invoke-Ssh([string]$cmd) { & "$env:WINDIR\System32\OpenSSH\ssh.exe" -o BatchMode=yes -o ConnectTimeout=8 "root@$Board" $cmd 2>&1 }
function ViewerWin { Get-Process powershell -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowTitle -like "beaconRace*" } }

if ($Status) {
    Say "board process / port / camera:"
    Invoke-Ssh "sh /home/root/br_ctl.sh status"
    $w = ViewerWin
    if ($w) { Say ("viewer window: " + ($w.MainWindowTitle -join "; ")) } else { Say "viewer window: none" }
    return
}

# 1) reachability
if (-not (Test-NetConnection $Board -Port 22 -InformationLevel Quiet -WarningAction SilentlyContinue)) {
    Say "cannot reach ${Board}:22 -- is the car's hotspot on and the PC joined it?"
    exit 1
}
Say "board $Board reachable"

# 2) drop the old viewer first: ImgStream serves only ONE client
$old = ViewerWin
if ($old) {
    $old | ForEach-Object { $_.CloseMainWindow() | Out-Null }
    Start-Sleep -Seconds 1
    ViewerWin | ForEach-Object { $_.Kill() }
    Say "closed $($old.Count) old viewer window(s)"
}

# 3) restart the board program in the requested mode
$flag = if ($Mode -eq 'drive') { '--drive' } else { '' }
Say "starting beaconRace $Mode on the board ..."
Invoke-Ssh "sh /home/root/br_ctl.sh start $flag"

# 4) wait until the stream port answers
$ok = $false
for ($i = 0; $i -lt 40; $i++) {
    if (Test-NetConnection $Board -Port $Port -InformationLevel Quiet -WarningAction SilentlyContinue) { $ok = $true; break }
    Start-Sleep -Milliseconds 250
}
if (-not $ok) { Say "port $Port never opened -- check /home/root/run_logs/br_run.log on the board"; exit 1 }
Say "stream port $Port is listening"

# 5) open the viewer
Start-Process -FilePath "powershell.exe" -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$Viewer`"")
Start-Sleep -Seconds 3
$w = ViewerWin
if ($w) { Say ("viewer up: " + ($w.MainWindowTitle -join "; ")) } else { Say "viewer window did not appear" }
Say "done. stop with:  powershell -File .\stop_all.ps1   (or press ESC in the window)"
