<#
.SYNOPSIS
    Download the open-access items of the L01-L22 source list into ref/,
    naming every file as <L-id>_<bibkey>.<ext>.

.DESCRIPTION
    Only gold / green / bronze / hybrid open-access items are downloaded.
    Subscription-only papers are NOT handled here; see MISSING.md.
    Existing files are skipped. PDFs are validated by magic bytes and size.

    ASCII-only on purpose: Windows PowerShell 5.1 reads .ps1 as ANSI unless the
    file carries a UTF-8 BOM, which would corrupt any non-ASCII text here.

.PARAMETER DryRun
    Print the plan; do not touch the network or the disk.

.PARAMETER Verify
    Read INDEX.csv and report which expected files are present, invalid or missing.

.PARAMETER OutDir
    Target directory. Defaults to the folder that contains this script.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File ref\fetch_refs.ps1 -DryRun
    powershell -ExecutionPolicy Bypass -File ref\fetch_refs.ps1
    powershell -ExecutionPolicy Bypass -File ref\fetch_refs.ps1 -Verify
#>
[CmdletBinding()]
param(
    [switch]$DryRun,
    [switch]$Verify,
    [string]$OutDir = ''
)

$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($OutDir)) {
    # $PSScriptRoot is not always bound while param defaults are evaluated.
    $OutDir = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }
}
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Force -Path $OutDir | Out-Null }
$logPath = Join-Path $OutDir 'download_log.txt'
$today = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'

# Open-access items only (OpenAlex status checked 2026-11-25).
$items = @(
    @{ Id='L01-3'; Key='wynn2014auvs';           Ext='pdf';  Url='https://www.sciencedirect.com/science/article/pii/S0025322714000747/pdf'; Hint='hybrid OA; if 403, open the DOI page on campus network' }
    @{ Id='L09-2'; Key='dechter1985generalized'; Ext='pdf';  Url='https://dl.acm.org/doi/pdf/10.1145/3828.3830'; Hint='ACM bronze; may need a browser session' }
    @{ Id='L11-1'; Key='geraerts2007creating';   Ext='pdf';  Url='https://dspace.library.uu.nl/handle/1874/31850'; Hint='repository landing page; PDF link is inside' }
    @{ Id='L12-1'; Key='fraichard2004reeds';     Ext='pdf';  Url='https://hal.inria.fr/inria-00000009/document'; Hint='HAL direct document' }
    @{ Id='L12-3'; Key='park2022three';          Ext='pdf';  Url='https://www.mdpi.com/2076-3417/12/22/11336/pdf?version=1668064367'; Hint='MDPI gold' }
    @{ Id='L14-2'; Key='bingul2023intelligent';  Ext='pdf';  Url='https://www.mdpi.com/2075-1702/11/2/300/pdf?version=1676625969'; Hint='MDPI gold' }
    @{ Id='L15-1'; Key='donald1993kinodynamic';  Ext='pdf';  Url='https://dl.acm.org/doi/pdf/10.1145/174147.174150'; Hint='ACM bronze' }
    @{ Id='L16-2'; Key='wang2024trajectory';     Ext='pdf';  Url='https://www.mdpi.com/2077-1312/12/8/1442/pdf?version=1724206375'; Hint='MDPI gold' }
    @{ Id='L17-1'; Key='kot2022review';          Ext='pdf';  Url='https://www.mdpi.com/2079-9292/11/15/2301/pdf?version=1659517501'; Hint='MDPI gold' }
    @{ Id='L18-2'; Key='buche2026lotusim';       Ext='pdf';  Url='https://arxiv.org/pdf/2607.03072'; Hint='arXiv id came from OpenAlex; verify on arxiv.org' }
    @{ Id='L20-1'; Key='sandve2013ten';          Ext='pdf';  Url='https://journals.plos.org/ploscompbiol/article/file?id=10.1371/journal.pcbi.1003285&type=printable'; Hint='PLOS gold' }
    @{ Id='L20-2'; Key='peng2011reproducible';   Ext='pdf';  Url='https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3383002/pdf/'; Hint='PMC green; filename varies, fall back to the article page' }
    @{ Id='L22-1'; Key='zhao2020sim';            Ext='pdf';  Url='https://arxiv.org/pdf/2009.13303'; Hint='arXiv green' }
    @{ Id='L22-2'; Key='tiboni2023dropo';        Ext='pdf';  Url='https://doi.org/10.1016/j.robot.2023.104432'; Hint='Elsevier hybrid OA; DOI resolves to the PDF page' }
    @{ Id='L21-1'; Key='noaa2024pm0101';         Ext='html'; Url='https://tidesandcurrents.noaa.gov/cdata/StationInfo?id=pm0101'; Hint='web page' }
    @{ Id='L21-2'; Key='noaa_coops_measure';     Ext='html'; Url='https://tidesandcurrents.noaa.gov/measure.html'; Hint='web page' }
)

# Public but not automatable: the direct link needs a human click.
$manual = @(
    @{ Id='L08-1'; Key='lavalle2006planning'; Why='free book on the author site; confirm the PDF link on the page'; Where='https://lavalle.pl/planning/' }
    @{ Id='L18-3'; Key='quigley2009ros';      Why='no DOI; community-hosted'; Where='search: "ROS: an open-source Robot Operating System" pdf' }
)

# Subscription-only, listed for completeness (see MISSING.md).
$closed = @(
    'L01-1','L01-2','L02-1','L02-2','L03-1','L03-2','L04-1','L04-2','L04-3','L05-1','L05-2',
    'L06-1','L06-2','L07-2','L07-3','L08-2','L09-1','L10-1','L10-2','L12-2','L13-1','L13-2',
    'L13-3','L14-1','L15-2','L16-1','L17-2','L18-1','L19-1','L19-2'
)

function Get-TargetName {
    param($Item)
    '{0}_{1}.{2}' -f $Item.Id, $Item.Key, $Item.Ext
}

function Test-PdfFile {
    param([string]$Path)
    if (-not (Test-Path $Path)) { return $false }
    $len = (Get-Item $Path).Length
    if ($len -lt 20000) { return $false }
    $fs = [System.IO.File]::OpenRead($Path)
    try {
        $buf = New-Object byte[] 4
        $null = $fs.Read($buf, 0, 4)
    } finally { $fs.Close() }
    return ([System.Text.Encoding]::ASCII.GetString($buf) -eq '%PDF')
}

Write-Host ("ref plan: {0} auto, {1} manual, {2} subscription" -f $items.Count, $manual.Count, $closed.Count) -ForegroundColor Cyan
Write-Host ("target dir: {0}" -f $OutDir)

if ($Verify) {
    $csv = Join-Path $OutDir 'INDEX.csv'
    if (-not (Test-Path $csv)) { throw "INDEX.csv not found in $OutDir" }
    $rows = Import-Csv -Path $csv -Encoding UTF8 | Where-Object { $_.status -ne 'DUP' -and $_.filename }
    $present = @(); $missing = @(); $invalid = @()
    foreach ($r in $rows) {
        $p = Join-Path $OutDir $r.filename
        if (-not (Test-Path $p)) { $missing += $r.id }
        elseif ($r.filename -like '*.pdf' -and -not (Test-PdfFile $p)) { $invalid += $r.id }
        else { $present += $r.id }
    }
    $total = ($rows | Measure-Object).Count
    Write-Host ("verify: {0}/{1} present, {2} invalid, {3} missing" -f $present.Count, $total, $invalid.Count, $missing.Count) -ForegroundColor Cyan
    if ($invalid.Count) { Write-Host ("  invalid (not a real PDF): " + ($invalid -join ', ')) -ForegroundColor Red }
    if ($missing.Count) { Write-Host ("  missing: " + ($missing -join ', ')) -ForegroundColor Yellow }
    return
}

if ($DryRun) {
    foreach ($it in $items) {
        Write-Host ("  [AUTO] {0,-46} {1}" -f (Get-TargetName $it), $it.Url)
    }
    foreach ($m in $manual) {
        Write-Host ("  [MAN ] {0}_{1}.pdf   {2} -> {3}" -f $m.Id, $m.Key, $m.Why, $m.Where) -ForegroundColor Yellow
    }
    Write-Host ("  [SUB ] " + ($closed -join ', ')) -ForegroundColor DarkGray
    return
}

$ok = 0; $fail = 0; $skip = 0
$ua = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124.0 Safari/537.36'

foreach ($it in $items) {
    $name = Get-TargetName $it
    $path = Join-Path $OutDir $name

    if (Test-Path $path) {
        Write-Host ("  skip  {0} (exists)" -f $name) -ForegroundColor DarkGray
        $skip++
        continue
    }

    try {
        Invoke-WebRequest -Uri $it.Url -OutFile $path -TimeoutSec 90 -UserAgent $ua -MaximumRedirection 10 | Out-Null
        if ($it.Ext -eq 'pdf' -and -not (Test-PdfFile $path)) {
            throw 'downloaded content is not a valid PDF (blocked or login page returned)'
        }
        $size = (Get-Item $path).Length
        Write-Host ("  ok    {0}  {1:N0} bytes" -f $name, $size) -ForegroundColor Green
        Add-Content $logPath ("{0}`tOK`t{1}`t{2}`t{3}" -f $today, $name, $size, $it.Url)
        $ok++
    }
    catch {
        $msg = $_.Exception.Message
        Write-Host ("  FAIL  {0}  {1}" -f $name, $msg) -ForegroundColor Red
        Write-Host ("        hint: {0}" -f $it.Hint) -ForegroundColor DarkYellow
        Add-Content $logPath ("{0}`tFAIL`t{1}`t0`t{2}`t{3}" -f $today, $name, $it.Url, $msg)
        if (Test-Path $path) { Remove-Item $path -Force }
        $fail++
    }
}

Write-Host ''
Write-Host ("done: ok={0} skip={1} fail={2}" -f $ok, $skip, $fail) -ForegroundColor Cyan
Write-Host ("log:  {0}" -f $logPath)
if ($fail -gt 0) {
    Write-Host ''
    Write-Host 'For failed items, download manually with the same file name and drop it in this folder.' -ForegroundColor Yellow
    Write-Host 'Subscription items: see ref/MISSING.md' -ForegroundColor Yellow
}
