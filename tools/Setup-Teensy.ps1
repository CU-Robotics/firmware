# Setup-Teensy.ps1 - Bind Teensy to WSL for flashing
# Defaults to Serial Mode (16c0:048b) and Bootloader Mode (16c0:0478)

#Requires -RunAsAdministrator

param(
    [int]$DetectTimeoutSec = 10,
    [string]$SerialVID = "16c0",
    [string]$SerialPID = "048b",
    [string]$SerialAltVID = "1fc9",
    [string]$SerialAltPID = "0135",
    [string]$BootloaderVID = "16c0",
    [string]$BootloaderPID = "0478"
)

function Get-TeensyBusId {
    param([string]$VendorId, [string]$ProductId)
    $dev = usbipd list 2>$null | Select-String "${VendorId}:${ProductId}"
    if ($dev) {
        return $dev.ToString().Trim().Split(' ')[0]
    }
    return $null
}

function Find-Device {
    param([string]$PrimaryVID, [string]$PrimaryPID, [string]$AltVID, [string]$AltPID)
    $busId = Get-TeensyBusId -VendorId $PrimaryVID -ProductId $PrimaryPID
    if ($busId) {
        return [pscustomobject]@{ BusId = $busId; Vid = $PrimaryVID; Pid = $PrimaryPID }
    }
    if ($AltVID -and $AltPID) {
        $busId = Get-TeensyBusId -VendorId $AltVID -ProductId $AltPID
        if ($busId) {
            return [pscustomobject]@{ BusId = $busId; Vid = $AltVID; Pid = $AltPID }
        }
    }
    return $null
}

function Get-DeviceIdsByBusId {
    param([string]$BusId)
    if (-not $BusId) {
        return $null
    }
    $line = usbipd list 2>$null | Select-String ("^" + [regex]::Escape($BusId) + "\s+")
    if (-not $line) {
        return $null
    }
    $match = [regex]::Match($line.ToString(), "\b([0-9A-Fa-f]{4}):([0-9A-Fa-f]{4})\b")
    if (-not $match.Success) {
        return $null
    }
    return [pscustomobject]@{ Vid = $match.Groups[1].Value; Pid = $match.Groups[2].Value }
}

function Wait-For-Device {
    param([string]$PrimaryVID, [string]$PrimaryPID, [string]$AltVID, [string]$AltPID, [string]$Label, [int]$TimeoutSec)
    $spinner = @('|', '/', '-', '\')
    $i = 0
    while ($TimeoutSec -gt 0) {
        $found = Find-Device -PrimaryVID $PrimaryVID -PrimaryPID $PrimaryPID -AltVID $AltVID -AltPID $AltPID
        if ($found) {
            if (-not $found.Vid -or -not $found.Pid) {
                $ids = Get-DeviceIdsByBusId -BusId $found.BusId
                if ($ids) {
                    $found.Vid = $ids.Vid
                    $found.Pid = $ids.Pid
                }
            }
            Write-Host "`r  $Label detected ($($found.Vid):$($found.Pid)) on Bus ID: $($found.BusId)                     " -ForegroundColor Green
            return $found
        }
        Write-Host -NoNewline "`r  Waiting for $Label $($spinner[$i % 4]) ($TimeoutSec`s) "
        Start-Sleep -Seconds 1
        $TimeoutSec--
        $i++
    }
    Write-Host ""
    return $null
}

function Bind-Device {
    param([string]$HardwareId, [string]$BusId, [string]$Name)
    if ($HardwareId -and ($HardwareId -notmatch "^\s*:\s*$")) {
        $output = usbipd bind --hardware-id $HardwareId 2>&1 | Out-String
    } elseif ($BusId) {
        $output = usbipd bind --busid $BusId 2>&1 | Out-String
    } else {
        Write-Host "  $Name - missing hardware-id and busid" -ForegroundColor Yellow
        return
    }
    if ($output -match "already shared") {
        Write-Host "  $Name already bound" -ForegroundColor DarkGray
    } elseif ($output -match "found at busid") {
        Write-Host "  $Name bound successfully" -ForegroundColor Green
    } else {
        Write-Host "  $Name - $output" -ForegroundColor Yellow
    }
}

function Is-Device-Attached {
    param([string]$BusId)
    if (-not $BusId) {
        return $false
    }
    $line = usbipd list 2>$null | Select-String ("^" + [regex]::Escape($BusId) + "\s+")
    if ($line -and $line.ToString() -match "Attached") {
        return $true
    }
    return $false
}

function Attach-Device {
    param([string]$BusId, [string]$Name)
    if (-not $BusId) {
        return
    }
    if (Is-Device-Attached -BusId $BusId) {
        Write-Host "  $Name already attached to WSL" -ForegroundColor DarkGray
        return
    }
    try {
        Start-Process -FilePath "usbipd" -ArgumentList @("attach", "--wsl", "--busid", $BusId, "--auto-attach") -WindowStyle Hidden | Out-Null
        Write-Host "  $Name auto-attach requested" -ForegroundColor Green
    } catch {
        Write-Host "  $Name - failed to start auto-attach: $($_.Exception.Message)" -ForegroundColor Yellow
    }
}

# Check if already in bootloader mode
$bootloaderBusId = Get-TeensyBusId -VendorId $BootloaderVID -ProductId $BootloaderPID
if ($bootloaderBusId) {
    Write-Host "Teensy already in Bootloader Mode (Bus ID: $bootloaderBusId)" -ForegroundColor Yellow
    Bind-Device -HardwareId "${BootloaderVID}:${BootloaderPID}" -BusId $bootloaderBusId -Name "Bootloader"
    $serialInfo = Find-Device -PrimaryVID $SerialVID -PrimaryPID $SerialPID -AltVID $SerialAltVID -AltPID $SerialAltPID
    if ($serialInfo) {
        if (-not $serialInfo.Vid -or -not $serialInfo.Pid) {
            $ids = Get-DeviceIdsByBusId -BusId $serialInfo.BusId
            if ($ids) {
                $serialInfo.Vid = $ids.Vid
                $serialInfo.Pid = $ids.Pid
            }
        }
        Write-Host "Found Teensy (Serial Mode $($serialInfo.Vid):$($serialInfo.Pid)) on Bus ID: $($serialInfo.BusId)" -ForegroundColor Green
        Bind-Device -HardwareId "$($serialInfo.Vid):$($serialInfo.Pid)" -BusId $serialInfo.BusId -Name "Serial Mode"
    }
    Attach-Device -BusId $serialInfo.BusId -Name "Serial Mode"
    Attach-Device -BusId $bootloaderBusId -Name "Bootloader"
    Write-Host "`nSetup complete! Auto-attach requested." -ForegroundColor Cyan
    Write-Host "`nThis window will close in 5 seconds..." -ForegroundColor DarkGray
    Start-Sleep -Seconds 5
    exit 0
}

# Look for serial mode
Write-Host "Searching for Teensy..." -ForegroundColor Cyan
$serialInfo = Wait-For-Device -PrimaryVID $SerialVID -PrimaryPID $SerialPID -AltVID $SerialAltVID -AltPID $SerialAltPID -Label "Serial Mode" -TimeoutSec $DetectTimeoutSec

if (-not $serialInfo) {
    Write-Host "Teensy not found. Ensure it's plugged in and visible in 'usbipd list'." -ForegroundColor Red
    Write-Host "`nThis window will close in 10 seconds..." -ForegroundColor DarkGray
    Start-Sleep -Seconds 10
    exit 1
}

Write-Host "Found Teensy (Serial Mode $($serialInfo.Vid):$($serialInfo.Pid)) on Bus ID: $($serialInfo.BusId)" -ForegroundColor Green
Bind-Device -HardwareId "$($serialInfo.Vid):$($serialInfo.Pid)" -BusId $serialInfo.BusId -Name "Serial Mode"

# Wait for bootloader
Write-Host "`nPress the Teensy button to enter Bootloader Mode..." -ForegroundColor Yellow
$timeout = 30
$spinner = @('|', '/', '-', '\')
$i = 0

while ($timeout -gt 0) {
    $currentDev = usbipd list 2>$null | Select-String $serialInfo.BusId
    if ($currentDev -match "${BootloaderVID}:${BootloaderPID}") {
        Write-Host "`r  Bootloader detected!                    " -ForegroundColor Green
        break
    }
    Write-Host -NoNewline "`r  Waiting $($spinner[$i % 4]) ($timeout`s) "
    Start-Sleep -Seconds 1
    $timeout--
    $i++
}

if ($timeout -eq 0) {
    Write-Host "`nTimeout waiting for bootloader. Did you press the button?" -ForegroundColor Red
    Write-Host "`nThis window will close in 10 seconds..." -ForegroundColor DarkGray
    Start-Sleep -Seconds 10
    exit 1
}

$bootloaderBusId = Get-TeensyBusId -VendorId $BootloaderVID -ProductId $BootloaderPID

Bind-Device -HardwareId "${BootloaderVID}:${BootloaderPID}" -BusId $bootloaderBusId -Name "Bootloader Mode"
Attach-Device -BusId $serialInfo.BusId -Name "Serial Mode"
Attach-Device -BusId $bootloaderBusId -Name "Bootloader Mode"

# Final output
Write-Host "`n$('=' * 50)" -ForegroundColor Cyan
Write-Host "Setup complete! Auto-attach requested for:" -ForegroundColor Cyan
Write-Host "  Serial Mode: $($serialInfo.BusId)" -ForegroundColor White
if ($bootloaderBusId) {
    Write-Host "  Bootloader Mode: $bootloaderBusId" -ForegroundColor White
}
Write-Host "$('=' * 50)" -ForegroundColor Cyan
Write-Host "`nThis window will close in 5 seconds..." -ForegroundColor DarkGray
Start-Sleep -Seconds 5
