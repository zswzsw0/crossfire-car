# One command: cross-compile on the VM -> pull to PC -> push to the car -> restart beaconRace
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\deploy.ps1            (default --no-drive)
#   powershell ... -File .\deploy.ps1 -Mode drive                               (car really drives)
#   powershell ... -File .\deploy.ps1 -NoRestart                                (swap binary only)
#   powershell ... -File .\deploy.ps1 -Restore                                  (roll back + restart)
#
# Every run backs the old board binary up to /home/root/beaconRace.bak.<timestamp>.
# ASCII-only on purpose: Windows PowerShell 5.1 reads a BOM-less .ps1 as ANSI,
# so any non-ASCII text in this file would corrupt the parse. Keep it ASCII.
param(
    [ValidateSet('no-drive', 'drive')][string]$Mode = 'no-drive',
    [switch]$NoRestart,
    [switch]$Restore,
    [string]$VmHost = 'mark@192.168.52.133',
    [string]$BoardHost = 'root@192.168.1.3',
    [string]$SerialDir = '/home/mark/serial',
    [string]$BinRel = 'build_la/beaconRace'
)
$ErrorActionPreference = 'Stop'

$SSH = Join-Path $env:WINDIR 'System32\OpenSSH\ssh.exe'
$SCP = Join-Path $env:WINDIR 'System32\OpenSSH\scp.exe'
$Stage = Join-Path $PSScriptRoot 'transfer'
New-Item -ItemType Directory -Force -Path $Stage | Out-Null

function Say([string]$m) { Write-Output ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $m) }
function Fail([string]$m) { Write-Output ("[FAIL] " + $m); exit 1 }

function SshRun([string]$target, [string]$cmd) {
    $out = & $SSH -o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new $target $cmd 2>&1
    return @{ Code = $LASTEXITCODE; Out = (($out | Out-String).Trim()) }
}
function ScpFile([string]$src, [string]$dst) {
    $out = & $SCP -O -q -o BatchMode=yes -o ConnectTimeout=10 $src $dst 2>&1
    return @{ Code = $LASTEXITCODE; Out = (($out | Out-String).Trim()) }
}
function Md5Local([string]$p) { (Get-FileHash -Path $p -Algorithm MD5).Hash.ToLower() }

# ---------------- rollback ----------------
if ($Restore) {
    Say 'rollback: looking for the newest backup on the board ...'
    $r = SshRun $BoardHost 'ls -t /home/root/beaconRace.bak.* 2>/dev/null | head -1'
    if (-not $r.Out) { Fail 'no backup found on board' }
    $bak = ($r.Out -split "`n")[-1].Trim()
    Say ("restore from " + $bak)
    $cmd = "pkill -x beaconRace 2>/dev/null; sleep 1; cp -p '$bak' /home/root/beaconRace && chmod +x /home/root/beaconRace && md5sum /home/root/beaconRace"
    $r = SshRun $BoardHost $cmd
    Say $r.Out
    if ($r.Code -ne 0) { Fail 'restore failed' }
    if (-not $NoRestart) {
        $flag = if ($Mode -eq 'drive') { '--drive' } else { '' }
        Say 'restarting ...'
        Say (SshRun $BoardHost "sh /home/root/br_ctl.sh start $flag").Out
    }
    Say 'done.'
    exit 0
}

# ---------------- 1. build on the VM ----------------
Say "1/5 cross-compile on VM ($VmHost) ..."
$r = SshRun $VmHost "cd '$SerialDir' && sh build_la.sh 2>&1 | tail -6"
Say $r.Out
if ($r.Code -ne 0) { Fail 'build failed on VM' }

$r = SshRun $VmHost "md5sum '$SerialDir/$BinRel'; stat -c %s '$SerialDir/$BinRel'"
$vmMd5 = ([regex]::Match($r.Out, '[0-9a-fA-F]{32}')).Value.ToLower()
$vmSize = ([regex]::Match($r.Out, '(?m)^\s*(\d+)\s*$')).Groups[1].Value
if (-not $vmMd5 -or -not $vmSize) { Fail ("cannot parse VM artifact info: " + $r.Out) }
Say ("   VM artifact: {0} bytes, md5 {1}" -f $vmSize, $vmMd5)

# ---------------- 2. pull to the PC ----------------
# Stage inside the VM first: a path without parentheses/spaces keeps legacy scp happy.
Say '2/5 pull artifact to PC ...'
$r = SshRun $VmHost "cp -f '$SerialDir/$BinRel' /tmp/beaconRace.out && chmod 644 /tmp/beaconRace.out && echo staged"
if ($r.Code -ne 0) { Fail ("staging on VM failed: " + $r.Out) }

$local = Join-Path $Stage 'beaconRace_new'
if (Test-Path $local) { Remove-Item $local -Force }
$r = ScpFile ("{0}:/tmp/beaconRace.out" -f $VmHost) $local
if ($r.Code -ne 0 -or -not (Test-Path $local)) { Fail ("scp from VM failed: " + $r.Out) }
$localMd5 = Md5Local $local
Say ("   local: {0} bytes, md5 {1}" -f (Get-Item $local).Length, $localMd5)
if ($localMd5 -ne $vmMd5) { Fail 'md5 mismatch VM -> PC' }

# ---------------- 3. push to the car ----------------
Say '3/5 push to the car ...'
$r = ScpFile $local "${BoardHost}:/home/root/beaconRace.new"
if ($r.Code -ne 0) { Fail ("scp to board failed: " + $r.Out) }
$r = SshRun $BoardHost 'chmod +x /home/root/beaconRace.new; md5sum /home/root/beaconRace.new'
$boardMd5 = ([regex]::Match($r.Out, '[0-9a-fA-F]{32}')).Value.ToLower()
if (-not $boardMd5) { Fail ("cannot read board md5: " + $r.Out) }
Say ("   board: md5 {0}" -f $boardMd5)
if ($boardMd5 -ne $localMd5) { Fail 'md5 mismatch PC -> board' }

# ---------------- 4. backup + swap ----------------
# The swap runs from an uploaded shell script: passing a multi-line heredoc through
# ssh.exe from PowerShell silently eats embedded double quotes (that once turned
# `echo "backup -> x"` into `echo backup -> x`, i.e. a redirect that truncated the
# freshly made backup). Uploading a .sh file sidesteps all of that.
Say '4/5 backup the old binary and swap in the new one ...'
$swapLocal = Join-Path $PSScriptRoot 'board_swap.sh'
if (-not (Test-Path $swapLocal)) { Fail "missing helper script: $swapLocal" }
$r = ScpFile $swapLocal "${BoardHost}:/tmp/board_swap.sh"
if ($r.Code -ne 0) { Fail ("upload swap script failed: " + $r.Out) }
$r = SshRun $BoardHost 'sh /tmp/board_swap.sh'
Say $r.Out
if ($r.Code -ne 0) { Fail 'swap failed on board' }

# ---------------- 5. restart ----------------
if ($NoRestart) {
    Say '5/5 restart skipped (-NoRestart)'
} else {
    $flag = if ($Mode -eq 'drive') { '--drive' } else { '' }
    Say ("5/5 restart beaconRace ($Mode) ...")
    $r = SshRun $BoardHost "sh /home/root/br_ctl.sh start $flag"
    Say $r.Out
    if ($r.Code -ne 0) { Fail 'restart failed' }
}

# tidy up the VM staging copy
SshRun $VmHost 'rm -f /tmp/beaconRace.out' | Out-Null
Say 'done.'
