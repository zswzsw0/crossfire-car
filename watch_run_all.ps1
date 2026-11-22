# Watch a whole beaconRace run without ever grabbing frames itself.
#
# It holds two long-lived SSH "tail -F" streams (one on the car, one on the VM viewer)
# and turns them into an event timeline:
#   board  : /home/root/run_logs/br_run.log        (state machine: WAIT_CARD / NAV_GOAL / DONE + pose)
#   viewer : /tmp/viewer_status.log (per-second stats the viewer computes from the
#                                    frames it already decoded -- no extra capture)
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\watch_run_all.ps1
#   ... -MaxSeconds 900
#
# Exit: 0 when DONE is seen, 2 on timeout.
# ASCII-only: PowerShell 5.1 parses BOM-less .ps1 as ANSI.
param(
    [int]$MaxSeconds = 600,
    [int]$PollMs = 800
)
$ErrorActionPreference = 'Stop'

$Board = 'root@192.168.1.3'
$Vm = 'mark@192.168.52.133'
$SSH = Join-Path $env:WINDIR 'System32\OpenSSH\ssh.exe'
$Work = Join-Path $PSScriptRoot 'transfer\streams'
New-Item -ItemType Directory -Force -Path $Work | Out-Null
$boardLog = Join-Path $Work 'board_stream.log'
$viewLog = Join-Path $Work 'viewer_stream.log'
$report = Join-Path $PSScriptRoot 'run_report.log'

function Stamp { (Get-Date -Format 'HH:mm:ss') }
function Emit([string]$tag, [string]$msg) {
    $line = "[{0}] {1,-10} {2}" -f (Stamp), $tag, $msg
    Write-Output $line
    Add-Content -Path $report -Value $line -Encoding UTF8
}

function Start-Tail([string]$target, [string]$remote, [string]$outFile) {
    if (Test-Path $outFile) { Remove-Item $outFile -Force }
    $args = @('-o', 'BatchMode=yes', '-o', 'ServerAliveInterval=10', '-o', 'ConnectTimeout=10',
              $target, "tail -n 0 -F $remote")
    return Start-Process -FilePath $SSH -ArgumentList $args -NoNewWindow -PassThru `
        -RedirectStandardOutput $outFile -RedirectStandardError (Join-Path $Work 'tail.err')
}

function Read-New([string]$path, [ref]$offset) {
    if (-not (Test-Path $path)) { return @() }
    try {
        $fs = [System.IO.File]::Open($path, 'Open', 'Read', 'ReadWrite')
    } catch { return @() }
    try {
        if ($fs.Length -le $offset.Value) { return @() }
        $fs.Seek($offset.Value, 'Begin') | Out-Null
        $sr = New-Object System.IO.StreamReader($fs, [System.Text.Encoding]::UTF8)
        $txt = $sr.ReadToEnd()
        $offset.Value = $fs.Position
    } finally { if ($sr) { $sr.Dispose() }; $fs.Dispose() }
    return ($txt -split "`n" | Where-Object { $_.Trim() -ne '' })
}

# ---------- start the two streams ----------
"# watch_run_all started $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" | Out-File $report -Encoding UTF8
Emit 'START' "streams: board=$Board  viewer=$Vm"
$pBoard = Start-Tail $Board '/home/root/run_logs/br_run.log' $boardLog
$pView = Start-Tail $Vm '/tmp/viewer_status.log' $viewLog
Start-Sleep -Seconds 2

$offBoard = [ref]0
$offView = [ref]0

# ---------- run state ----------
$state = 'UNKNOWN'
$prevState = 'UNKNOWN'
$card = '-'
$lastPose = ''
$poseChanges = 0
$maxRed = 0.0
$maxBlue = 0.0
$cardVisibleAt = $null
$recognizedAt = $null
$doneAt = $null
$lastViewSeconds = -1
$viewStaleCount = 0
$lastHeartbeat = Get-Date
$t0 = Get-Date
$trkSamples = 0
$trkHits = 0
$lastBlk = 0
$lastVx = '0'

Emit 'ARMED' 'waiting for the car to report WAIT_CARD and for the card to show up'

while (((Get-Date) - $t0).TotalSeconds -lt $MaxSeconds) {
    # ---- board lines ----
    foreach ($l in (Read-New $boardLog $offBoard)) {
        # 板端 tail 可能先补发已有内容：见到启动横幅就把计时与状态全部重置，
        # 否则会把上一轮/本轮的旧行当成本次进度（曾误报「2.5s 到终点」）。
        if ($l -match 'beaconRace 启动') {
            $t0 = Get-Date
            $state = 'UNKNOWN'; $prevState = 'UNKNOWN'
            $recognizedAt = $null; $doneAt = $null; $cardVisibleAt = $null
            $lastPose = ''; $poseChanges = 0
            Emit 'RESET' 'seen startup banner, timer restarted'
        }
        if ($l -match '^(WAIT_CARD|NAV_GOAL|DONE|PARK|STOP)\b') {
            $state = $Matches[1]
            if ($state -ne $prevState) {
                Emit 'STATE' ("{0} -> {1}" -f $prevState, $state)
                if ($state -ne 'WAIT_CARD' -and -not $recognizedAt) {
                    $recognizedAt = Get-Date
                    Emit 'CARD_OK' ("state machine left WAIT_CARD after {0:N1}s" -f ($recognizedAt - $t0).TotalSeconds)
                }
                $prevState = $state
            }
        }
        if ($l -match 'card=(blue|red)') { $card = $Matches[1] }
        # diagnostics added to the board log: blk=<n> trk=<0|1> cone=<n> and vx=<val>
        if ($l -match 'trk=(\d)') {
            $trkSamples++
            if ($Matches[1] -eq '1') { $trkHits++ }
        }
        if ($l -match 'blk=(\d+)') { $lastBlk = [int]$Matches[1] }
        if ($l -match 'vx=([-\d.]+)') { $lastVx = $Matches[1] }
        if ($l -match 'pose=\(([-\d.]+),([-\d.]+)\)') {
            $pose = "$($Matches[1]),$($Matches[2])"
            if ($pose -ne $lastPose) {
                if ($lastPose -ne '') { $poseChanges++ }
                $lastPose = $pose
            }
        }
        if ($l -match '^DONE' -and -not $doneAt) {
            $doneAt = Get-Date
            Emit 'DONE' ("reached goal in {0:N1}s  card={1}  pose=({2})" -f ($doneAt - $t0).TotalSeconds, $card, $lastPose)
        }
        if ($l -match 'exit|ERROR|failed') { Emit 'PROBLEM' $l.Trim() }
    }

    # ---- viewer lines (positional parse: the log has Chinese labels) ----
    foreach ($l in (Read-New $viewLog $offView)) {
        $t = $l.Trim() -split '\s+'
        # ts status fps 红 X% 蓝 Y% 绿 Z% 亮度 L 变化 D%
        if ($t.Count -ge 13) {
            $r = $t[4] -replace '%', ''
            $b = $t[6] -replace '%', ''
            if ($r -match '^[\d.]+$' -and $b -match '^[\d.]+$') {
                $rv = [double]$r; $bv = [double]$b
                if ($rv -gt $maxRed) { $maxRed = $rv }
                if ($bv -gt $maxBlue) { $maxBlue = $bv }
                if (-not $cardVisibleAt -and ($rv -gt 20 -or $bv -gt 20)) {
                    $cardVisibleAt = Get-Date
                    Emit 'CARD_SEEN' ("red={0:N1}% blue={1:N1}% -- card entered the view" -f $rv, $bv)
                }
            }
        }
    }

    if ($doneAt) { break }

    # ---- heartbeat every 10s ----
    if (((Get-Date) - $lastHeartbeat).TotalSeconds -ge 10) {
        $lastHeartbeat = Get-Date
        $trkPct = if ($trkSamples -gt 0) { 100.0 * $trkHits / $trkSamples } else { 0 }
        Emit 'BEAT' ("state={0} card={1} pose=({2}) maxRed={3:N1}% maxBlue={4:N1}% moves={5} | trk={6:N0}% ({7}/{8}) blk={9} vx={10}" -f $state, $card, $lastPose, $maxRed, $maxBlue, $poseChanges, $trkPct, $trkHits, $trkSamples, $lastBlk, $lastVx)
        $trkSamples = 0
        $trkHits = 0
    }

    Start-Sleep -Milliseconds $PollMs
}

# ---------- wrap up ----------
foreach ($p in @($pBoard, $pView)) { try { if ($p -and -not $p.HasExited) { $p.Kill() } } catch {} }

# ---------- archive the car's log onto this PC (kept off the car's /tmp) ----------
$seconds = [int]((Get-Date) - $t0).TotalSeconds
$save = Join-Path $PSScriptRoot 'save_run_log.ps1'
if (Test-Path $save) {
    $res = if ($doneAt) { 'DONE' } else { 'TIMEOUT' }
    $cardArg = if ($card -and $card -ne '-') { $card } else { 'unknown' }
    try {
        & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $save `
            -Card $cardArg -Result $res -Seconds $seconds `
            -Note ("watcher run in report {0}" -f $report) 2>&1 |
            ForEach-Object { Emit 'ARCHIVE' $_ }
    } catch {
        Emit 'ARCHIVE' ("log archive failed: " + $_.Exception.Message)
    }
}

if ($doneAt) {
    Emit 'SUMMARY' ("PASS expected: DONE  card={0}  final pose=({1})  maxRed={2:N1}%  maxBlue={3:N1}%  pose changes={4}" -f $card, $lastPose, $maxRed, $maxBlue, $poseChanges)
    exit 0
} else {
    $hint = 'card never showed up in the frame'
    if ($cardVisibleAt) { $hint = 'card was visible but the state machine did not move on (thresholds / frames / dominance)' }
    Emit 'SUMMARY' ("TIMEOUT state={0} card={1} pose=({2}) maxRed={3:N1}% maxBlue={4:N1}% -- {5}" -f $state, $card, $lastPose, $maxRed, $maxBlue, $hint)
    exit 2
}
