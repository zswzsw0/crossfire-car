# Capture one frame from the board's ImgStream and save it as PNG (verification helper).
# ASCII-only on purpose so Windows PowerShell 5.1 parses it regardless of file encoding.
# Usage: powershell -NoProfile -ExecutionPolicy Bypass -File .\capture_frame.ps1 [-Host_ 192.168.1.3] [-Port 9999] [-Out <path>]
param(
    [string]$Host_ = "192.168.1.3",
    [int]$Port = 9999,
    [string]$Out = (Join-Path $env:TEMP "cf_frame.png")
)

Add-Type -AssemblyName System.Drawing

$client = New-Object System.Net.Sockets.TcpClient
$client.ReceiveTimeout = 8000
try { $client.Connect($Host_, $Port) } catch {
    Write-Output ("CONNECT_FAIL {0}:{1} {2}" -f $Host_, $Port, $_.Exception.Message); exit 1
}
$s = $client.GetStream()

$header = New-Object byte[] 4
$got = 0
while ($got -lt 4) { $n = $s.Read($header, $got, 4 - $got); if ($n -le 0) { Write-Output "DISCONNECTED_HEADER"; exit 1 }; $got += $n }
$len = ([int]$header[0] -shl 24) -bor ([int]$header[1] -shl 16) -bor ([int]$header[2] -shl 8) -bor [int]$header[3]
Write-Output ("FRAME_BYTES {0}" -f $len)

$jpg = New-Object byte[] $len
$got = 0
while ($got -lt $len) { $n = $s.Read($jpg, $got, $len - $got); if ($n -le 0) { Write-Output "DISCONNECTED_BODY"; exit 1 }; $got += $n }

$ms = New-Object System.IO.MemoryStream(,$jpg)
$img = [System.Drawing.Image]::FromStream($ms)
Write-Output ("FRAME_SIZE {0}x{1}" -f $img.Width, $img.Height)
$img.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
Write-Output ("SAVED {0}" -f $Out)
$img.Dispose(); $ms.Dispose(); $s.Close(); $client.Close()
