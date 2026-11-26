# Capture the car camera stream to a frame sequence while a run happens.
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\capture_run.ps1
#   ... -MaxSeconds 300 -SaveEvery 3 -OutDir D:\br_frames
#
# The board stream server (beaconRace, TCP 9999) serves ONE client at a time,
# so the VM viewer (gtk_viewer.py) must be stopped while this runs.
#
# ASCII-ONLY ON PURPOSE: PowerShell 5.1 parses a BOM-less .ps1 as ANSI, so any
# non-ASCII literal in this file (including a Chinese default path) becomes
# mojibake. Keep every literal here ASCII, including default values.
param(
    [string]$HostAddr = '192.168.1.3',
    [int]$Port = 9999,
    [string]$OutDir = 'D:\br_frames',
    [int]$MaxSeconds = 300,
    [int]$SaveEvery = 3,
    [int]$MaxSave = 900
)
$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Get-ChildItem $OutDir -Filter *.jpg -ErrorAction SilentlyContinue | Remove-Item -Force

$tcp = New-Object System.Net.Sockets.TcpClient
$tcp.Connect($HostAddr, $Port)
$st = $tcp.GetStream()
$st.ReadTimeout = 15000
$br = New-Object System.IO.BinaryReader($st)

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$seen = 0
$saved = 0
$idx = 0
while ($sw.Elapsed.TotalSeconds -lt $MaxSeconds -and $saved -lt $MaxSave) {
    $lenBytes = $br.ReadBytes(4)
    if ($lenBytes.Length -lt 4) { break }
    $len = ([int]$lenBytes[0] -shl 24) -bor ([int]$lenBytes[1] -shl 16) -bor ([int]$lenBytes[2] -shl 8) -bor [int]$lenBytes[3]
    if ($len -le 0 -or $len -gt 4000000) { break }
    $img = $br.ReadBytes($len)
    if ($img.Length -lt $len) { break }
    $seen++
    if (($seen % $SaveEvery) -eq 0) {
        $idx++
        $name = Join-Path $OutDir ('f{0:D4}_{1:00.0}s.jpg' -f $idx, $sw.Elapsed.TotalSeconds)
        [System.IO.File]::WriteAllBytes($name, $img)
        $saved++
    }
}
$tcp.Close()
"seen=$seen saved=$saved elapsed=$([math]::Round($sw.Elapsed.TotalSeconds,1))s -> $OutDir"
