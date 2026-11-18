# PC-side viewer for beaconRace ImgStream -- zero dependency (GDI+ / WinForms only).
# No numpy / opencv needed. Works with Windows PowerShell 5.1 (powershell.exe).
# Usage: powershell -NoProfile -ExecutionPolicy Bypass -File .\viewer.ps1 [-Host_ 192.168.1.3] [-Port 9999]
#   ESC or closing the window stops it. Title bar shows bytes/frame and FPS.
param(
    [string]$Host_ = "192.168.1.3",
    [int]$Port = 9999
)

Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$client = New-Object System.Net.Sockets.TcpClient
try { $client.Connect($Host_, $Port) } catch {
    Write-Output ("CONNECT_FAIL {0}:{1} {2}" -f $Host_, $Port, $_.Exception.Message)
    [System.Windows.Forms.MessageBox]::Show(("Cannot connect to {0}:{1}`nStart on board: cd /home/root && ./start.sh --no-drive" -f $Host_, $Port), "beaconRace viewer") | Out-Null
    exit 1
}
$stream = $client.GetStream()
Write-Output ("CONNECTED {0}:{1}" -f $Host_, $Port)

$form = New-Object System.Windows.Forms.Form
$form.Text = ("beaconRace {0}:{1}" -f $Host_, $Port)
$form.Width = 660
$form.Height = 540
$form.StartPosition = "CenterScreen"
$form.KeyPreview = $true

$pic = New-Object System.Windows.Forms.PictureBox
$pic.Dock = "Fill"
$pic.SizeMode = [System.Windows.Forms.PictureBoxSizeMode]::Zoom
$form.Controls.Add($pic)
$form.Add_KeyDown({ if ($_.KeyCode -eq "Escape") { $form.Close() } })

$header = New-Object byte[] 4
$frames = 0
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$closed = $false

$form.Add_Shown({
    while (-not $closed -and $form.Visible) {
        try {
            if (-not $stream.DataAvailable) {
                [System.Windows.Forms.Application]::DoEvents()
                Start-Sleep -Milliseconds 4
                continue
            }
            $got = 0
            while ($got -lt 4) {
                $n = $stream.Read($header, $got, 4 - $got)
                if ($n -le 0) { throw "stream closed (header)" }
                $got += $n
            }
            $len = ([int]$header[0] -shl 24) -bor ([int]$header[1] -shl 16) -bor ([int]$header[2] -shl 8) -bor [int]$header[3]
            if ($len -le 0 -or $len -gt 8MB) { throw "bad frame length $len" }

            $jpg = New-Object byte[] $len
            $got = 0
            while ($got -lt $len) {
                $n = $stream.Read($jpg, $got, $len - $got)
                if ($n -le 0) { throw "stream closed (body)" }
                $got += $n
            }

            $ms = New-Object System.IO.MemoryStream(,$jpg)
            $img = [System.Drawing.Image]::FromStream($ms)
            $old = $pic.Image
            $pic.Image = $img
            if ($old) { $old.Dispose() }
            $ms.Dispose()

            $frames++
            if ($sw.Elapsed.TotalSeconds -ge 1) {
                $fps = [math]::Round($frames / $sw.Elapsed.TotalSeconds, 1)
                $form.Text = "beaconRace {0}:{1}  {2}B  {3}fps" -f $Host_, $Port, $len, $fps
                $frames = 0
                $sw.Restart()
            }
            [System.Windows.Forms.Application]::DoEvents()
        } catch {
            $form.Text = "disconnected: " + $_.Exception.Message
            break
        }
    }
})
$form.Add_FormClosed({ $closed = $true; try { $stream.Close() } catch {}; try { $client.Close() } catch {} })

[System.Windows.Forms.Application]::Run($form)
