# Save one run of the car's log to this PC (and into git).
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\save_run_log.ps1
#   ... -Card blue -Result DONE -Seconds 18 -Note "goal 6.90 / tol 0.10"
#
# What it does:
#   run_logs/<timestamp>_<card>_<result>[_<secs>s].log   raw /home/root/run_logs/br_run.log from the car
#   run_logs/<same>.cfg                                 the exact config used
#   run_logs/<same>.meta.txt                            md5 + summary + note
#   then commits run_logs/ so the archive does not rot as untracked files.
#
# ASCII-only on purpose: Windows PowerShell 5.1 parses a BOM-less .ps1 as ANSI.
param(
    [string]$BoardHost = 'root@192.168.1.3',
    [string]$Card = 'unknown',
    [string]$Result = 'unknown',
    [int]$Seconds = 0,
    [string]$Note = '',
    [switch]$NoCommit
)
$ErrorActionPreference = 'Stop'

$SSH  = Join-Path $env:WINDIR 'System32\OpenSSH\ssh.exe'
$SCP  = Join-Path $env:WINDIR 'System32\OpenSSH\scp.exe'
$Repo = $PSScriptRoot
$Dir  = Join-Path $Repo 'run_logs'
New-Item -ItemType Directory -Force -Path $Dir | Out-Null

function Say([string]$m) { Write-Output ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $m) }
function Ssh([string]$cmd) {
    $o = & $SSH -o BatchMode=yes -o ConnectTimeout=8 $BoardHost $cmd 2>&1
    return @{ Code = $LASTEXITCODE; Out = (($o | Out-String).Trim()) }
}

# ---- reachability first: never hang the caller on an offline car ----
# Use a real ssh probe instead of Test-NetConnection: the latter can report
# false negatives (and is slow) when the interface is a hotspot client.
$probe = & $SSH -o BatchMode=yes -o ConnectTimeout=6 $BoardHost 'echo OK' 2>&1
if ($LASTEXITCODE -ne 0 -or ("$probe" -notmatch 'OK')) {
    Say "car unreachable ($BoardHost) - log NOT saved"
    exit 1
}

# ---- build the file name ----
$stamp = Get-Date -Format 'yyyy-MM-dd_HHmmss'
$safe = { param($s) ($s -replace '[^\w\-]', '_') }
$name = "{0}_{1}_{2}" -f $stamp, (& $safe $Card), (& $safe $Result)
if ($Seconds -gt 0) { $name += ("_{0}s" -f $Seconds) }
$logPath  = Join-Path $Dir ($name + '.log')
$cfgPath  = Join-Path $Dir ($name + '.cfg')
$metaPath = Join-Path $Dir ($name + '.meta.txt')

# ---- pull the log and the config that produced it ----
Say "pulling log from $BoardHost ..."
& $SCP -O -q -o BatchMode=yes -o ConnectTimeout=8 "${BoardHost}:/home/root/run_logs/br_run.log" $logPath
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $logPath)) {
    Say "scp of /home/root/run_logs/br_run.log failed - nothing saved"
    exit 1
}
& $SCP -O -q -o BatchMode=yes -o ConnectTimeout=8 "${BoardHost}:/home/root/beacon_race.cfg" $cfgPath 2>$null

# ---- collect facts about the run ----
$md5 = (Ssh 'md5sum /home/root/beaconRace').Out
if ($md5 -match '[0-9a-fA-F]{32}') { $md5 = $Matches[0].ToLower() } else { $md5 = 'unknown' }

$lines = Get-Content $logPath
$final = ($lines | Where-Object { $_ -match '^DONE ' } | Select-Object -Last 1)
$firstNav = ($lines | Where-Object { $_ -match '^NAV_GOAL ' } | Select-Object -First 1)
$spins = ($lines | Where-Object { $_ -match '^NAV_GOAL .*vx=0 ' }).Count
$navs  = ($lines | Where-Object { $_ -match '^NAV_GOAL ' }).Count
$maxVx = ($lines | ForEach-Object { if ($_ -match 'vx=([0-9.]+)') { [double]$Matches[1] } } | Measure-Object -Maximum).Maximum

# error against the goal, when both the pose and the cfg are known
$err = ''
if ($final -and (Test-Path $cfgPath)) {
    $cfg = Get-Content $cfgPath
    if ($final -match 'card=(blue|red)') {
        $c = $Matches[1]
        $gx = ($cfg | Where-Object { $_ -match "^${c}_goal_x=" }) -replace "^${c}_goal_x=", ''
        $gy = ($cfg | Where-Object { $_ -match "^${c}_goal_y=" }) -replace "^${c}_goal_y=", ''
        if ($final -match 'pose=\(([-\d.]+),([-\d.]+)\)' -and $gx -and $gy) {
            $dx = [double]$gx - [double]$Matches[1]
            $dy = [double]$gy - [double]$Matches[2]
            $err = ('{0:N3} m' -f [math]::Sqrt($dx * $dx + $dy * $dy))
        }
    }
}

# ---- write the companion meta file ----
$meta = @(
    "run      : $name",
    "saved    : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
    "source   : ${BoardHost}:/home/root/run_logs/br_run.log",
    "binary   : md5 $md5",
    "card     : $Card",
    "result   : $Result",
    "seconds  : $Seconds",
    "nav_goal : $navs lines, $spins in-place spins",
    "max_vx   : $maxVx",
    "final    : $final",
    "error    : $err",
    "note     : $Note"
) -join "`r`n"
[System.IO.File]::WriteAllText($metaPath, $meta + "`r`n", (New-Object System.Text.UTF8Encoding($false)))

Say ("saved -> run_logs\{0}.log ({1} bytes)" -f $name, (Get-Item $logPath).Length)
if ($err) { Say ("goal error: {0};  final: {1}" -f $err, $final) }

if (-not $NoCommit) {
    Push-Location $Repo
    # git writes harmless warnings (e.g. CRLF) to stderr; PowerShell would turn
    # those into terminating errors under $ErrorActionPreference = 'Stop'.
    $prevEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & git add -- run_logs *> $null
        $msg = "运行日志存档 $name`n`ncard=$Card result=$Result seconds=$Seconds error=$err`n$Note"
        & git commit -q -m $msg *> $null
        $code = $LASTEXITCODE
        if ($code -eq 0) { Say 'committed to git' } else { Say 'nothing new to commit (or git busy)' }
    } catch {
        Say ("git step failed: " + $_.Exception.Message)
    } finally {
        $ErrorActionPreference = $prevEap
        Pop-Location
    }
}
exit 0