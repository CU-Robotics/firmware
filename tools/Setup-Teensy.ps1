# Setup-Teensy.ps1 - Bind Teensy to WSL for flashing
# Handles both Serial Mode (048b) and Bootloader Mode (0478)

#Requires -RunAsAdministrator

$TeensyVID = "16c0"
$SerialPID = "048b"
$BootloaderPID = "0478"

function Get-TeensyBusId {
    param([string]$ProductId)
    $dev = usbipd list 2>$null | Select-String "${TeensyVID}:${ProductId}"
    if ($dev) {
        return $dev.ToString().Trim().Split(' ')[0]
    }
    return $null
}

function Bind-Device {
    param([string]$HardwareId, [string]$Name)
    $output = usbipd bind --hardware-id $HardwareId 2>&1 | Out-String
    if ($output -match "already shared") {
        Write-Host "  $Name already bound" -ForegroundColor DarkGray
    } elseif ($output -match "found at busid") {
        Write-Host "  $Name bound successfully" -ForegroundColor Green
    } else {
        Write-Host "  $Name - $output" -ForegroundColor Yellow
    }
}

# Check if already in bootloader mode
$bootloaderBusId = Get-TeensyBusId -ProductId $BootloaderPID
if ($bootloaderBusId) {
    Write-Host "Teensy already in Bootloader Mode (Bus ID: $bootloaderBusId)" -ForegroundColor Yellow
    Bind-Device -HardwareId "${TeensyVID}:${BootloaderPID}" -Name "Bootloader"
    Write-Host "`nReady! Run:" -ForegroundColor Cyan
    Write-Host "  usbipd attach --wsl --busid $bootloaderBusId --auto-attach" -ForegroundColor White
    exit 0
}

# Look for serial mode
Write-Host "Searching for Teensy..." -ForegroundColor Cyan
$serialBusId = Get-TeensyBusId -ProductId $SerialPID

if (-not $serialBusId) {
    Write-Host "Teensy not found. Ensure it's plugged in and not in bootloader mode." -ForegroundColor Red
    exit 1
}

Write-Host "Found Teensy (Serial Mode) on Bus ID: $serialBusId" -ForegroundColor Green
Bind-Device -HardwareId "${TeensyVID}:${SerialPID}" -Name "Serial Mode"

# Wait for bootloader
Write-Host "`nPress the Teensy button to enter Bootloader Mode..." -ForegroundColor Yellow
$timeout = 30
$spinner = @('|', '/', '-', '\')
$i = 0

while ($timeout -gt 0) {
    $currentDev = usbipd list 2>$null | Select-String $serialBusId
    if ($currentDev -match "${TeensyVID}:${BootloaderPID}") {
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
    exit 1
}

Bind-Device -HardwareId "${TeensyVID}:${BootloaderPID}" -Name "Bootloader Mode"

# Final output
Write-Host "`n$('=' * 50)" -ForegroundColor Cyan
Write-Host "Setup complete! Run this to auto-attach:" -ForegroundColor Cyan
Write-Host "  usbipd attach --wsl --busid $serialBusId --auto-attach" -ForegroundColor White
Write-Host "$('=' * 50)" -ForegroundColor Cyan