# Analyze black-dot blobs in a captured frame using the SAME HSV rules as beaconRace.
# Thresholds from hsv_saved.txt:  HUE_BLACK = (0, 0, 0, 180, 80, 60)  -> S<=80, V<=60
# Geometry filters from source: min_area 20, max_area 700, w>=8, h>=6, aspect 0.2..2.2
# ASCII-only (PowerShell 5.1 / ANSI parse safety).
param(
    [string]$Png = (Join-Path $env:TEMP 'cf_frame.png'),
    [int]$SMax = 80,
    [int[]]$VMaxList = @(60, 75, 90),
    [int]$AreaMin = 20,
    [int]$AreaMax = 700
)

Add-Type -AssemblyName System.Drawing

$img = [System.Drawing.Bitmap]::new($Png)
$W = $img.Width; $H = $img.Height
Write-Output ("frame {0}x{1}" -f $W, $H)

# cache pixel components once
$R = New-Object 'int[]' ($W * $H)
$G = New-Object 'int[]' ($W * $H)
$B = New-Object 'int[]' ($W * $H)
for ($y = 0; $y -lt $H; $y++) {
    for ($x = 0; $x -lt $W; $x++) {
        $c = $img.GetPixel($x, $y)
        $i = $y * $W + $x
        $R[$i] = $c.R; $G[$i] = $c.G; $B[$i] = $c.B
    }
}
$img.Dispose()

$roiY = [int]($H * 0.45)   # roi_y_top = 0.45 -> rows above this are ignored

foreach ($VMax in $VMaxList) {
    # build mask
    $mask = New-Object 'bool[]' ($W * $H)
    $n = 0
    for ($i = 0; $i -lt ($W * $H); $i++) {
        $y = [int][math]::Floor($i / $W)
        if ($y -lt $roiY) { continue }          # zeroRoi
        $r = $R[$i]; $g = $G[$i]; $b = $B[$i]
        $mx = [math]::Max($r, [math]::Max($g, $b))
        $mn = [math]::Min($r, [math]::Min($g, $b))
        $v = $mx
        $s = if ($mx -eq 0) { 0 } else { 255.0 * ($mx - $mn) / $mx }
        if ($v -le $VMax -and $s -le $SMax) { $mask[$i] = $true; $n++ }
    }

    # connected components (4-neighbour, iterative)
    $seen = New-Object 'bool[]' ($W * $H)
    $areas = New-Object 'System.Collections.ArrayList'
    $pass = 0
    $stack = New-Object 'System.Collections.Generic.Stack[int]'
    $minx = 0; $maxx = 0; $miny = 0; $maxy = 0
    for ($i = 0; $i -lt ($W * $H); $i++) {
        if (-not $mask[$i] -or $seen[$i]) { continue }
        $stack.Clear(); $stack.Push($i); $seen[$i] = $true
        $cnt = 0
        $x0 = [int]($i % $W); $y0 = [int][math]::Floor($i / $W)
        $minx = $x0; $maxx = $x0; $miny = $y0; $maxy = $y0
        while ($stack.Count -gt 0) {
            $p = $stack.Pop(); $cnt++
            $px = $p % $W; $py = [int][math]::Floor($p / $W)
            if ($px -lt $minx) { $minx = $px }; if ($px -gt $maxx) { $maxx = $px }
            if ($py -lt $miny) { $miny = $py }; if ($py -gt $maxy) { $maxy = $py }
            foreach ($d in @(-1, 1, -$W, $W)) {
                $q = $p + $d
                if ($q -lt 0 -or $q -ge ($W * $H)) { continue }
                if ($d -eq 1 -and ($px + 1 -ge $W)) { continue }
                if ($d -eq -1 -and ($px - 1 -lt 0)) { continue }
                if ($mask[$q] -and -not $seen[$q]) { $seen[$q] = $true; $stack.Push($q) }
            }
        }
        $w = $maxx - $minx + 1
        $h = $maxy - $miny + 1
        $asp = $h / [double]$w
        $ok = ($cnt -ge $AreaMin) -and ($cnt -le $AreaMax) -and ($w -ge 8) -and ($h -ge 6) -and ($asp -ge 0.20) -and ($asp -le 2.2)
        if ($cnt -ge $AreaMin) {
            [void]$areas.Add([pscustomobject]@{ Area = $cnt; W = $w; H = $h; Asp = [math]::Round($asp, 2); Pass = $ok })
        }
        if ($ok) { $pass++ }
    }

    Write-Output ""
    Write-Output ("=== V<={0}  S<={1} : 掩膜像素 {2}   连通域>=minArea {3}   通过全部几何过滤 {4} ===" -f $VMax, $SMax, $n, $areas.Count, $pass)
    if ($areas.Count -gt 0) {
        $hist = $areas | Group-Object { 
            if ($_.Area -lt 50) { '20-49' } elseif ($_.Area -lt 150) { '50-149' } elseif ($_.Area -lt 400) { '150-399' }
            elseif ($_.Area -lt 700) { '400-699' } elseif ($_.Area -lt 1500) { '700-1499' } else { '>=1500' }
        } | Select-Object Name, Count | Sort-Object Name
        $hist | Format-Table -AutoSize | Out-String | Write-Output
        Write-Output ("  最大的 5 个: " + (($areas | Sort-Object Area -Descending | Select-Object -First 5 | ForEach-Object { "area=$($_.Area) ${($_.W)}x${($_.H)} asp=$($_.Asp)" }) -join ' | '))
        $big = ($areas | Where-Object { $_.Area -gt $AreaMax }).Count
        Write-Output ("  超过 max_area={0} 被丢弃的: {1} 个" -f $AreaMax, $big)
    }
}
