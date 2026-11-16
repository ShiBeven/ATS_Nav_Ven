$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$utf8 = New-Object System.Text.UTF8Encoding($false)
$source = [IO.File]::ReadAllText((Join-Path $projectRoot 'application\usb_task.c'))
$original = [IO.File]::ReadAllText((Join-Path $PSScriptRoot '修改前\usb_task.c'))
$start = 'static void unified_nav_apply_ats_gcc_54('
$end = 'static void unified_nav_apply_50('
if ($source.Substring(0, $source.IndexOf($start)) -cne $original.Substring(0, $original.IndexOf($start))) { throw 'Unexpected change before parser' }
if ($source.Substring($source.IndexOf($end)) -cne $original.Substring($original.IndexOf($end))) { throw 'Unexpected change after parser' }
$tail = '  nav_last_rx = now;'
$oldFunction = $original.Substring($original.IndexOf($start), $original.IndexOf($end) - $original.IndexOf($start))
$newFunction = $source.Substring($source.IndexOf($start), $source.IndexOf($end) - $source.IndexOf($start))
if ($newFunction.Substring($newFunction.IndexOf($tail)) -cne $oldFunction.Substring($oldFunction.IndexOf($tail))) { throw 'Navigation/vision control changed' }
if ((Get-FileHash -LiteralPath (Join-Path $projectRoot 'application\usb_task.h')).Hash -ne (Get-FileHash -LiteralPath (Join-Path $PSScriptRoot '修改前\usb_task.h')).Hash) { throw 'USB header changed' }

function Get-CFunction([string]$code, [string]$name) {
    $match = [regex]::Match($code, ('(?m)^(?:static\s+)?[\w* ]+\b' + [regex]::Escape($name) + '\([^;]*?\)\s*\{'))
    if (-not $match.Success) { throw "Missing function: $name" }
    $open = $code.IndexOf('{', $match.Index)
    $depth = 1
    $close = $open + 1
    while ($depth -gt 0) {
        if ($code[$close] -eq '{') { $depth++ }
        if ($code[$close] -eq '}') { $depth-- }
        $close++
    }
    return $code.Substring($match.Index, $close - $match.Index)
}
$clean = [regex]::Replace($source, '/\*.*?\*/|//[^\n]*', '', [Text.RegularExpressions.RegexOptions]::Singleline)
$hostCode = [IO.File]::ReadAllText((Join-Path $projectRoot '验证\视觉接口协议对齐\usb_protocol_host_test.cpp'))
foreach ($name in @('Get_CRC16_Check_Sum', 'Verify_CRC16_Check_Sum', 'Append_CRC16_Check_Sum', 'unified_usb_receive_bytes', 'unified_crc8', 'unified_nav_apply_ats_gcc_54', 'unified_nav_apply_50', 'usb_apply_vision_command', 'unified_usb_frame', 'unified_usb_process', 'usb_send_vision', 'unified_usb_send_state', 'unified_usb_send_imu_state', 'unified_usb_send_joint_state')) {
    $hostCode = $hostCode.Replace((Get-CFunction $hostCode $name), (Get-CFunction $clean $name))
}
# Host stubs represent ARMCC's one-byte enum using uint8_t. Supply its tag
# for the explicit firmware cast; the full firmware build checks the real ABI.
$hostCode = $hostCode.Replace('#pragma pack(push, 1)', "enum mode { Move = 0, Attack = 1, Defend = 2 };`n#pragma pack(push, 1)")
$hostCode = $hostCode.Replace('  uint32_t before;', '  uint8_t expected54[52];' + "`n" + '  uint32_t before;')
$anchor = '    CHECK(nav_override == 1 && RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode == 2);'
if (-not $hostCode.Contains($anchor)) { throw '54-byte regression anchor missing' }
$extra = @'
    memset(expected54, 0, sizeof(expected54));
    memcpy(expected54, nav54, 21);
    expected54[21] = nav54[21];
    memcpy(expected54 + 22, nav54 + 25, 27);
    memcpy(expected54 + 50, nav54 + 52, 2);
    CHECK(memcmp(&RECEIVE_ROBOT_CMD_DATA, expected54, 52) == 0);
'@
$hostCode = $hostCode.Replace($anchor, $anchor + "`n" + $extra)
$hostPath = Join-Path $PSScriptRoot 'usb_protocol_host_test.cpp'
[IO.File]::WriteAllText($hostPath, $hostCode, $utf8)
$vcRoot = 'C:\Program Files (x86)\Microsoft Visual Studio\VC98'
$env:PATH = "$vcRoot\Bin;$(Split-Path $vcRoot -Parent)\Common\MSDev98\Bin;$env:PATH"
$env:INCLUDE = "$vcRoot\Include"
$env:LIB = "$vcRoot\Lib"
Push-Location $PSScriptRoot
try {
    $buildOutput = & "$vcRoot\Bin\CL.EXE" /nologo /TP /MLd /W3 $hostPath '/Feusb_protocol_host_test.exe' 2>&1
    $buildExit = $LASTEXITCODE
    [IO.File]::WriteAllText((Join-Path $PSScriptRoot 'host-build.log'), ($buildOutput -join "`r`n"), $utf8)
    if ($buildExit -ne 0) { throw ($buildOutput -join "`n") }
    $testOutput = & '.\usb_protocol_host_test.exe' 2>&1
    $testExit = $LASTEXITCODE
    [IO.File]::WriteAllText((Join-Path $PSScriptRoot 'host-test.log'), ($testOutput -join "`r`n"), $utf8)
    if ($testExit -ne 0) { throw ($testOutput -join "`n") }
    $testOutput
    'PASS: source outside parser, parser control tail and USB header unchanged'
} finally {
    Pop-Location
}
