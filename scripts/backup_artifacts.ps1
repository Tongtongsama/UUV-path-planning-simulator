param(
    [Parameter(Mandatory=$true)][string]$Destination
)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$sourceRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'artifacts'))
$backupRoot = [IO.Path]::GetFullPath($Destination)
if ($backupRoot.StartsWith($projectRoot.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Backup destination must be outside the project.'
}
if (Test-Path -LiteralPath $backupRoot) { throw 'Destination exists; no overwrite permitted.' }
$sourceItems = @(Get-ChildItem -LiteralPath $sourceRoot -Recurse -Force)
$links=@($sourceItems | Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint })
foreach ($link in $links) {
    if (-not $link.PSIsContainer -or $link.LinkType -ne 'Junction' -or
        $link.FullName.Substring($sourceRoot.Length+1) -ne 'presentations_build\node_modules') {
        throw 'Unexpected reparse point; review before backup.'
    }
}
$sourceFiles = @($sourceItems | Where-Object { -not $_.PSIsContainer -and -not ($_.Attributes -band [IO.FileAttributes]::ReparsePoint) } | Sort-Object FullName)
if ($sourceFiles.Count -eq 0) { throw 'Source is empty.' }
$entries = @()
foreach ($item in $sourceFiles) {
    $entries += [pscustomobject]@{
        path = $item.FullName.Substring($sourceRoot.Length + 1)
        bytes = $item.Length
        sha256 = (Get-FileHash -LiteralPath $item.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}
[IO.Directory]::CreateDirectory((Join-Path $backupRoot 'artifacts')) | Out-Null
foreach ($entry in $entries) {
    $target = Join-Path (Join-Path $backupRoot 'artifacts') $entry.path
    [IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($target)) | Out-Null
    [IO.File]::Copy((Join-Path $sourceRoot $entry.path), $target, $false)
}
$afterFiles = @(Get-ChildItem -LiteralPath $sourceRoot -Recurse -Force -File | Sort-Object FullName)
if ($afterFiles.Count -ne $entries.Count) { throw 'Source file inventory changed during backup.' }
for ($i=0; $i -lt $entries.Count; $i++) {
    $entry=$entries[$i]
    if ($afterFiles[$i].FullName.Substring($sourceRoot.Length+1) -ne $entry.path) {
        throw 'Source paths changed during backup.'
    }
    $sourceHash=(Get-FileHash -LiteralPath $afterFiles[$i].FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    $targetHash=(Get-FileHash -LiteralPath (Join-Path (Join-Path $backupRoot 'artifacts') $entry.path) -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($sourceHash -ne $entry.sha256 -or $targetHash -ne $entry.sha256) {
        throw "Hash verification failed: $($entry.path)"
    }
}
$indexSource=Join-Path $projectRoot 'docs/envelope_dataset_v1.json'
$indexTarget=Join-Path $backupRoot 'envelope_dataset_v1.json'
[IO.File]::Copy($indexSource,$indexTarget,$false)
if ((Get-FileHash -LiteralPath $indexSource).Hash -ne (Get-FileHash -LiteralPath $indexTarget).Hash) {
    throw 'Dataset index copy failed verification.'
}
$record=[ordered]@{
    status='VERIFIED'
    created_utc=[DateTime]::UtcNow.ToString('o')
    source=$sourceRoot
    destination=$backupRoot
    file_count=$entries.Count
    total_bytes=($entries | Measure-Object -Property bytes -Sum).Sum
    dataset_index_sha256=(Get-FileHash -LiteralPath $indexTarget).Hash.ToLowerInvariant()
    verification='SHA-256 source before and after copy, destination per-file, inventory unchanged'
    storage_note='Local backup on another drive letter; no claim of separate physical disk or off-site redundancy.'
    junction_policy='External dependency directory is recorded as metadata only; external cache contents are not followed or claimed to be backed up.'
    junctions=@($links | ForEach-Object { [ordered]@{path=$_.FullName.Substring($sourceRoot.Length+1);type=$_.LinkType;target=$_.Target} })
    files=$entries
}
$manifest=Join-Path $backupRoot 'backup_manifest.json'
[IO.File]::WriteAllText($manifest,($record | ConvertTo-Json -Depth 6),[Text.UTF8Encoding]::new($false))
Write-Output ([pscustomobject]@{status='VERIFIED';destination=$backupRoot;files=$entries.Count;bytes=$record.total_bytes;manifest_sha256=(Get-FileHash -LiteralPath $manifest).Hash.ToLowerInvariant()} | ConvertTo-Json)
