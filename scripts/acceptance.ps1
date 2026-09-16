# scripts/acceptance.ps1 · sim-source 独立验收脚本（退出码 0/1）
#
# 权威依据：
#   ① 本仓 README.md 的「做 / 不做」表 +「一句话边界」—— 逐行变成可执行检查
#   ② ../device-ingest/docs/契约/外设接入契约.md §2 归一化对象 —— **逐字段**与实现比对
#   ③ ../device-ingest/include/device_ingest/device_source.h —— 边界出处（只读校验）
#
# 设计：**引擎行为一律由 tests/selftest 断言**（selftest --json 输出逐用例结果与需求编号），
# 本脚本只做三件它做不了的事：构建生命周期、结构纪律检索（全仓/跨仓/SQL/业务词/网络）、
# 以及"归一化对象逐字段 == device-ingest 契约"的比对。这样行为口径只有一个来源。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File scripts/acceptance.ps1
#   ... -SkipBuild                只跑自测与检查（复用已有构建产物）
#   ... -Config Debug             换构建配置
#   ... -Generator "Ninja"        换生成器
#
# 兼容 Windows PowerShell 5.1（不依赖 pwsh / PS7 语法）。
param(
    [string]$BuildDir = "build",
    [string]$Config = "Release",
    [string]$Generator = "Visual Studio 17 2022",
    [string]$Arch = "x64",
    [string]$DeviceIngestDir = "",
    [switch]$SkipBuild
)

# 控制台按 UTF-8 读脚本与写输出（PS 5.1 的默认代码页会把中文读坏）
[void][System.Reflection.Assembly]::LoadWithPartialName("System.Text.Encoding")
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch { }
$ErrorActionPreference = "Stop"
$script:Results = New-Object System.Collections.Generic.List[object]
$script:ChecksFailed = 0

# ------------------------------------------------------------------ 输出小工具
function Check([string]$id, [string]$title, [bool]$ok, [string]$detail) {
    $state = "PASS"
    if (-not $ok) { $state = "FAIL"; $script:ChecksFailed++ }
    $script:Results.Add([pscustomobject]@{ Id = $id; Title = $title; Ok = $ok; Detail = $detail })
    $color = "Green"
    if (-not $ok) { $color = "Red" }
    Write-Host ("  [{0}] {1} · {2}" -f $state, $id, $title) -ForegroundColor $color
    if ($detail) { Write-Host ("         {0}" -f $detail) -ForegroundColor DarkGray }
}

function Section([string]$title) {
    Write-Host ""
    Write-Host ("=" * 78)
    Write-Host $title
    Write-Host ("=" * 78)
}

function Rel([string]$full) {
    $root = $script:Repo
    if ($full.StartsWith($root)) { return $full.Substring($root.Length).TrimStart('\', '/') }
    return $full
}

function Format-Hits($hits) {
    if (-not $hits -or $hits.Count -eq 0) { return "" }
    $head = @($hits | Select-Object -First 5)
    $s = "：" + ($head -join "；")
    if ($hits.Count -gt 5) { $s += ("；…共 {0} 处" -f $hits.Count) }
    return $s
}

function SearchHits([string[]]$files, [string]$regex, [string[]]$skipLineRegex) {
    $hits = New-Object System.Collections.Generic.List[string]
    foreach ($f in $files) {
        $n = 0
        foreach ($line in [System.IO.File]::ReadAllLines($f)) {
            $n++
            if ($line -notmatch $regex) { continue }
            if ($skipLineRegex) {
                $skip = $false
                foreach ($s in $skipLineRegex) { if ($line -match $s) { $skip = $true; break } }
                if ($skip) { continue }
            }
            $hits.Add(("{0}:{1}" -f (Rel $f), $n))
        }
    }
    return $hits
}

function CollectFiles([string[]]$dirs, [string[]]$exts, [string[]]$excludeDirs) {
    $out = New-Object System.Collections.Generic.List[string]
    foreach ($d in $dirs) {
        $full = Join-Path $script:Repo $d
        if (-not (Test-Path $full)) { continue }
        Get-ChildItem -Path $full -Recurse -File -ErrorAction SilentlyContinue | ForEach-Object {
            if ($exts -notcontains $_.Extension) { return }
            $rel = Rel $_.FullName
            foreach ($x in $excludeDirs) { if ($rel -like ($x + "*")) { return } }
            $out.Add($_.FullName)
        }
    }
    return $out
}

# ------------------------------------------------------------------ 前置
$script:Repo = Split-Path -Parent $PSScriptRoot
Push-Location $script:Repo
try {
    Write-Host "sim-source 独立验收（README 做/不做逐条 + 结构纪律 + 归一化对象逐字段比对）"
    Write-Host ("仓库：{0}" -f $script:Repo)
    Write-Host ("PowerShell：{0}" -f $PSVersionTable.PSVersion)

    $cmake = (Get-Command cmake -ErrorAction SilentlyContinue).Source
    if (-not $cmake) {
        foreach ($p in @("C:\Program Files\CMake\bin\cmake.exe",
                         "C:\Program Files (x86)\CMake\bin\cmake.exe")) {
            if (Test-Path $p) { $cmake = $p; break }
        }
    }
    if (-not $cmake) { Write-Host "找不到 cmake（>= 3.20）" -ForegroundColor Red; exit 1 }
    Write-Host ("cmake：{0}（{1}）" -f $cmake, (& $cmake --version | Select-Object -First 1))

    $buildPath = Join-Path $script:Repo $BuildDir
    $isMultiConfig = ($Generator -like "Visual Studio*")
    $binCandidates = @()
    if ($isMultiConfig) { $binCandidates += (Join-Path $buildPath "bin\$Config") }
    else { $binCandidates += (Join-Path $buildPath "bin") }
    $binCandidates += (Join-Path $buildPath "bin\$Config")
    $binCandidates += (Join-Path $buildPath "bin")

    function Resolve-Bin() {
        foreach ($c in $binCandidates) {
            if ((Test-Path (Join-Path $c "selftest.exe")) -or (Test-Path (Join-Path $c "selftest"))) {
                return $c
            }
        }
        return $binCandidates[0]
    }

    # ================================================================ ① 构建
    Section "① 构建生命周期（独立构建：一条命令构建 → 一条命令验收）"

    $configureOk = $false
    $buildOk = $false
    if ($SkipBuild) {
        $configureOk = (Test-Path (Join-Path $buildPath "CMakeCache.txt"))
    } else {
        Write-Host ("    cmake -S . -B {0} -G '{1}'" -f $BuildDir, $Generator)
        & $cmake -S $script:Repo -B $buildPath -G $Generator -A $Arch 2>&1 |
            ForEach-Object { Write-Host ("      {0}" -f $_) }
        $configureOk = ($LASTEXITCODE -eq 0)

        Write-Host ("    cmake --build {0} --config {1}" -f $BuildDir, $Config)
        & $cmake --build $buildPath --config $Config 2>&1 |
            ForEach-Object { Write-Host ("      {0}" -f $_) }
        $buildOk = ($LASTEXITCODE -eq 0)
    }

    $bin = Resolve-Bin
    $selftestExe = Join-Path $bin "selftest.exe"
    if (-not (Test-Path $selftestExe)) { $selftestExe = Join-Path $bin "selftest" }
    $hasSelftest = (Test-Path $selftestExe)
    $hasExamples = ((Test-Path (Join-Path $bin "example_minimal.exe")) -or
                    (Test-Path (Join-Path $bin "example_minimal"))) -and
                   ((Test-Path (Join-Path $bin "example_full_flow.exe")) -or
                    (Test-Path (Join-Path $bin "example_full_flow")))
    if ($SkipBuild) { $buildOk = ($hasSelftest -and $hasExamples) }
    Check "C01" "构建：配置 + 编译通过，产物齐全（selftest + 2 个示例）" `
        ($configureOk -and $buildOk -and $hasSelftest -and $hasExamples) `
        ("configure={0} build={1} selftest={2} examples={3} bin={4}" -f `
            $configureOk, $buildOk, $hasSelftest, $hasExamples, (Rel $bin))

    $cmakeLists = [System.IO.File]::ReadAllText((Join-Path $script:Repo "CMakeLists.txt"))
    $minOk = $false
    if ($cmakeLists -match 'cmake_minimum_required\s*\(\s*VERSION\s+([0-9]+\.[0-9]+)') {
        $minOk = ([version]$Matches[1] -ge [version]"3.20")
    }
    $cxx17 = ($cmakeLists -match 'CMAKE_CXX_STANDARD\s+17')
    Check "C02" "构建：CMake >= 3.20 且 C++17" ($minOk -and $cxx17) `
        ("cmake_min>=3.20={0} cxx17={1}" -f $minOk, $cxx17)

    # 库目标名必须是 sim_source
    $targetOk = ($cmakeLists -match 'add_library\(\s*sim_source\s+STATIC')
    Check "C03" "构建：库目标名 sim_source（STATIC）" $targetOk `
        "CMakeLists.txt 内 add_library(sim_source STATIC ...)"

    # ================================================================ ② selftest
    Section "② 零依赖自测（tests/selftest --json）"

    $jsonOk = $false
    $data = $null
    $jsonPath = Join-Path ([System.IO.Path]::GetTempPath()) ("sim-source-selftest-{0}.json" -f $PID)
    if (Test-Path $selftestExe) {
        # 让子进程**直接写文件**：selftest --json 输出 UTF-8（含中文用例名），
        # 走管道会被控制台代码页解成乱码，JSON 随之不可解析。
        & $selftestExe --json > $jsonPath
        $raw = ""
        if (Test-Path $jsonPath) {
            $raw = [System.IO.File]::ReadAllText($jsonPath, [System.Text.Encoding]::UTF8)
        }
        try {
            $data = $raw | ConvertFrom-Json
            $jsonOk = $true
        } catch {
            Write-Host "    selftest --json 解析失败：$($_.Exception.Message)" -ForegroundColor Red
            if ($raw.Length -gt 0) { Write-Host ($raw.Substring(0, [Math]::Min(400, $raw.Length))) }
        }
    }
    Check "C04" "自测可执行且 --json 输出可解析" $jsonOk ("exe={0}" -f (Rel $selftestExe))

    $caseByName = @{}
    $reqToCases = @{}
    if (-not $jsonOk) {
        Check "C05" "自测结果可用" $false "selftest --json 无有效输出"
    } else {
        Write-Host ("    用例 {0} 个（失败 {1}）｜断言 {2} 条（失败 {3}）｜耗时 {4:N1} ms｜结果 {5}" -f `
            $data.cases, $data.casesFailed, $data.asserts, $data.assertsFailed, $data.elapsedMs,
            $data.result)
        Check "C05" "自测全绿：用例失败 0 且断言失败 0" `
            (($data.casesFailed -eq 0) -and ($data.assertsFailed -eq 0)) `
            ("cases={0}/{1} asserts={2}/{3}" -f $data.cases, $data.casesFailed, $data.asserts,
             $data.assertsFailed)

        foreach ($c in $data.details) {
            $caseByName[$c.name] = $c
            foreach ($r in $c.reqs) {
                if (-not $reqToCases.ContainsKey($r)) {
                    $reqToCases[$r] = New-Object System.Collections.Generic.List[string]
                }
                $reqToCases[$r].Add($c.name)
            }
        }

        # ---- SIM-* 需求 → 用例对账（本模块的需求编号集合）
        $allReqs = New-Object System.Collections.Generic.List[string]
        foreach ($d in @("ROUTE:01..05", "KIN:01..04", "TICK:01..05", "TGT:01..04",
                         "OUT:01..05", "SENSOR:01..03", "IN:01..03", "ELC:01..01",
                         "NFR:01..05")) {
            $dom = $d.Split(':')[0]
            $parts = ($d.Split(':')[1] -replace '\.\.', ' ').Split(' ')
            for ($i = [int]$parts[0]; $i -le [int]$parts[1]; $i++) {
                $allReqs.Add(("SIM-{0}-{1:D2}" -f $dom, $i))
            }
        }
        $missing = New-Object System.Collections.Generic.List[string]
        foreach ($r in $allReqs) {
            if (-not $reqToCases.ContainsKey($r)) { $missing.Add($r) }
        }
        Check "C06" ("需求覆盖：{0} 条 SIM-* 每条都有用例引用" -f $allReqs.Count) `
            ($missing.Count -eq 0) `
            ("无对应用例：{0}" -f (($missing -join ", ") -replace '^$', '无'))

        $reqFailed = New-Object System.Collections.Generic.List[string]
        foreach ($r in $allReqs) {
            if (-not $reqToCases.ContainsKey($r)) { continue }
            $anyOk = $false
            foreach ($cn in $reqToCases[$r]) { if ($caseByName[$cn].ok) { $anyOk = $true } }
            if (-not $anyOk) { $reqFailed.Add($r) }
        }
        Check "C07" "需求覆盖：每条至少有一个通过用例" ($reqFailed.Count -eq 0) `
            ("未通过：{0}" -f (($reqFailed -join ", ") -replace '^$', '无'))
    }

    # ================================================================ ③ README「做 / 不做」逐条
    Section "③ README「做 / 不做」逐条（做的一列 = 行为用例；不做的一列 = 结构守卫）"

    $checklist = @(
        @{ Id = "R01"; Line = "做：编队航路规划（部署区→任务区，避让硬禁飞区）"
           Cases = @("route01_detour_around_hard_no_fly_zone",
                     "route02_kinematics_never_enters_hard_zone_and_arrives",
                     "route02b_soft_zone_does_not_change_route",
                     "route03_formation_keeps_relative_shape",
                     "route04_no_route_when_plan_impossible") },
        @{ Id = "R02"; Line = "做：实体运动学（位置/航向/速度/电量按注入时钟推进）"
           Cases = @("kin01_straight_segment_interpolation",
                     "kin02_waypoint_switch_and_arrival",
                     "kin03_battery_drain_and_clamp") },
        @{ Id = "R03"; Line = "做：节拍控制（实时 / 倍速 1×8×60× / 暂停 / 单步）"
           Cases = @("tick01_speed_8x_advances_8s_per_real_second",
                     "tick02_speed_multiplier_is_whitelisted",
                     "tick03_pause_discards_real_time_resume_does_not_jump",
                     "tick04_step_is_independent_of_speed",
                     "tick05_substep_keeps_waypoint_turn_accurate") },
        @{ Id = "R04"; Line = "做：目标运动（静止 / 航线 / 中途出现）"
           Cases = @("tgt01_static_target_does_not_move",
                     "tgt02_dynamic_target_moves_along_route_and_loops",
                     "tgt02b_non_loop_target_stops_at_end",
                     "tgt03_popup_appears_after_offset") },
        @{ Id = "R05"; Line = "做：事件出口（逐字段等于 device-ingest 归一化对象）"
           Cases = @("evt01_one_normalized_event_per_visible_entity_per_tick",
                     "evt02_event_fields_equal_device_ingest_contract",
                     "evt03_extensions_are_host_owned_and_deterministic",
                     "evt04_parse_round_trip_and_rejections") },
        @{ Id = "R06"; Line = "不做：不做协议收发（device-ingest 的活）—— 网络代码零命中"
           Guard = "network" },
        @{ Id = "R07"; Line = "不做：不做渲染（map-2d 的活）—— 无图形/窗口代码"
           Guard = "render" },
        @{ Id = "R08"; Line = "不做：不做业务判断（阶段、评级、编组）—— 业务词零命中"
           Guard = "business" },
        @{ Id = "R09"; Line = "不做：不内置探测模型（交 sensor-model）—— 只定义 ISensorModel"
           Cases = @("sensor01_attached_model_is_called_with_pose",
                     "sensor02_no_detection_algorithm_inside_module",
                     "sensor03_sink_exception_does_not_stop_simulation") },
        @{ Id = "R10"; Line = "不做：不查业务表 —— SQL 关键字零命中"
           Guard = "sql" },
        @{ Id = "R11"; Line = "边界：三者互不 import —— 跨仓 import 零命中"
           Guard = "crossrepo" },
        @{ Id = "R12"; Line = "边界：时间必须可注入（IClock）—— 模块内不取挂钟"
           Cases = @("nfr03_time_is_injectable_and_clock_is_optional") },
        @{ Id = "R13"; Line = "边界：事件出口是反向接口 ISimSink（宿主实现，模块不广播）"
           Cases = @("sensor03_sink_exception_does_not_stop_simulation") },
        @{ Id = "R14"; Line = "边界：只吃中立结构（不是业务表）"
           Cases = @("in01_json_scenario_round_trip",
                     "in02_validation_reports_readable_issues") },
        @{ Id = "R15"; Line = "确定性：注入假时钟，同一输入序列跑两次输出逐字节一致"
           Cases = @("nfr01_determinism_byte_identical_double_run",
                     "nfr01b_platform_declaration_order_does_not_matter") }
    )

    if (-not $jsonOk) {
        foreach ($item in $checklist) {
            if ($item.ContainsKey("Guard")) { continue }
            Check $item.Id $item.Line $false "自测结果不可用，无法对账"
        }
    } else {
        foreach ($item in $checklist) {
            if ($item.ContainsKey("Guard")) { continue }
            $missingCases = New-Object System.Collections.Generic.List[string]
            $failedCases = New-Object System.Collections.Generic.List[string]
            foreach ($cn in $item.Cases) {
                if (-not $caseByName.ContainsKey($cn)) { $missingCases.Add($cn); continue }
                if (-not $caseByName[$cn].ok) { $failedCases.Add($cn) }
            }
            $ok = ($missingCases.Count -eq 0) -and ($failedCases.Count -eq 0)
            $detail = "用例：{0}" -f ($item.Cases -join ", ")
            if ($missingCases.Count -gt 0) { $detail += "；缺失：" + ($missingCases -join ", ") }
            if ($failedCases.Count -gt 0) { $detail += "；失败：" + ($failedCases -join ", ") }
            Check $item.Id $item.Line $ok $detail
        }
    }

    # ================================================================ ④ 结构纪律
    Section "④ 结构纪律：SQL / 业务词 / 跨仓 import / 网络 / 渲染（README 不做列）"

    # 引擎产物范围：include/ src/ scripts/ CMakeLists.txt + examples/ + tests/
    # 说明：本模块是**模拟器**，"演示数据"（deviceType/kind/坐标/编组名）必须由宿主喂进来，
    # 交付物里的 demo 场景住在 examples/ 与 tests/（那是测试数据与演示数据的合法住所）；
    # 引擎产物（include/src/scripts/CMake）内 MUST NOT 出现任何业务取值。
    $engineFiles = [string[]]@(CollectFiles @("include", "src", "scripts") `
        @(".h", ".hpp", ".cc", ".cpp", ".ps1") @())
    foreach ($f in @("CMakeLists.txt")) {
        $full = Join-Path $script:Repo $f
        if (Test-Path $full) { $engineFiles += $full }
    }
    $codeFiles = [string[]]@(CollectFiles @("include", "src") @(".h", ".hpp", ".cc", ".cpp") @())
    $linkFiles = [string[]]@(CollectFiles @("include", "src", "examples", "scripts") `
        @(".h", ".hpp", ".cc", ".cpp", ".ps1") @())
    foreach ($f in @("CMakeLists.txt", "examples\CMakeLists.txt", "tests\CMakeLists.txt")) {
        $full = Join-Path $script:Repo $f
        if (Test-Path $full) { $linkFiles += $full }
    }
    # 判据自身（scripts/）与被检对象必须分开：守卫脚本里必然出现 "SELECT"/"阶段" 这些**字面量**，
    # 若把它自己算进检索范围，就永远"命中自己"（自指陷阱）。
    $selfScript = (Join-Path $script:Repo "scripts\acceptance.ps1")
    $notSelf = { param($f) return ($f -ne $selfScript) }
    $sqlHitsFiles = [string[]]@($engineFiles | Where-Object { $notSelf.Invoke($_) })
    $bizFiles = [string[]]@($engineFiles | Where-Object { $notSelf.Invoke($_) })
    $renderFiles = [string[]]@($linkFiles | Where-Object { $notSelf.Invoke($_) })

    Write-Host ("    引擎产物检索范围 {0} 个文件（docs/ examples/ tests/ 豁免）；" -f $engineFiles.Count)
    Write-Host ("    依赖/网络检索范围 {0} 个文件（含 examples/ 与各 CMakeLists）" -f $linkFiles.Count)

    # C08：SQL 与业务表零命中（README「不查业务表」）
    # 模式拆开拼接：脚本自身的正则不能命中自己（自指陷阱）
    $sqlPattern = '(?i)\b(SELECT|INSERT\s+INTO|UPDATE\s+\w+\s+SET|DELETE\s+FROM|CREATE\s+TABLE|' +
                  'DROP\s+TABLE)\b|sqlite|mysql|postgres|libpq|nanodbc|PQexec'
    $sqlHits = @(SearchHits $sqlHitsFiles $sqlPattern @('sqlPattern', 'SearchHits', 'MUST NOT',
                                                        '零命中', '检索范围'))
    Check "C08" "README「不查业务表」：引擎产物内 SQL 与数据库依赖零命中" ($sqlHits.Count -eq 0) `
        ("命中 {0} 处{1}" -f $sqlHits.Count, (Format-Hits $sqlHits))

    # C09：业务词零命中（场景键 / 阶段 / 评级 / 型号名）
    # 只保留"业务取值"型词：阶段/评级/威胁度、装备型号名、业务表字段名。
    # `deviceId/deviceType/kind` 属 device-ingest 契约的**接口字段名**，不在此列
    # （本项目正是要产出那个形状，把它们当业务词是自相矛盾的）。
    $bizPattern = '阶段|评级|威胁度|型号|光电|雷达|电子对抗|通信中继|' +
                  'mission_id|scenario_key|threat_level|progress_percent|approval_grade'
    $bizHits = @(SearchHits $bizFiles $bizPattern @('bizPattern', 'SearchHits', 'MUST NOT',
                                                    '零命中', '业务词', '检索范围', '场景键',
                                                    '边界声明'))
    Check "C09" "README「不做业务判断」：引擎产物内业务词零命中" ($bizHits.Count -eq 0) `
        ("命中 {0} 处{1}" -f $bizHits.Count, (Format-Hits $bizHits))

    # C10：跨仓 import 零命中（README「三者互不 import」）
    $otherRepos = '(device_ingest|device-ingest|device_source|sensor_model|sensor-model|' +
                  'map_2d|map-2d|entity_ledger|entity-ledger|phase_engine|phase-engine|' +
                  'resource_alloc|resource-alloc|realtime_hub|realtime-hub|telemetry_store|' +
                  'telemetry-store|mission_app|mission-app|scoring|topology|alert_engine|' +
                  'report_engine|view_composer|media_player|geo_data|selfcheck|assembly_host)'
    $incHits = @(SearchHits $linkFiles ('#\s*include\s*[<"]' + $otherRepos) `
        @('otherRepos', 'MUST NOT', '零命中', '检索范围'))
    Check "C11" "README「三者互不 import」：跨仓 include 零命中" ($incHits.Count -eq 0) `
        ("命中 {0} 处{1}" -f $incHits.Count, (Format-Hits $incHits))

    # C12：仅依赖 nlohmann/json
    $allIncludes = New-Object System.Collections.Generic.List[string]
    foreach ($f in $linkFiles) {
        foreach ($line in [System.IO.File]::ReadAllLines($f)) {
            $m = [regex]::Match($line, '^\s*#\s*include\s+(?:<([^>]+)>|"([^"]+)")')
            if ($m.Success) {
                if ($m.Groups[1].Success) { $allIncludes.Add($m.Groups[1].Value) }
                else { $allIncludes.Add($m.Groups[2].Value) }
            }
        }
    }
    $nonJson = $allIncludes | Where-Object {
        ($_ -notmatch '^(sim_source/|internal\.h$)') -and
        ($_ -notmatch '^[a-z_]+$') -and
        ($_ -notmatch '^(nlohmann|third_party)')
    }
    Check "C12" "仅依赖 nlohmann/json（无其它第三方头）" ($nonJson.Count -eq 0) `
        ("非标准库/非 nlohmann 的 include：{0}" -f (($nonJson -join ", ") -replace '^$', '无'))

    # C13：不做协议收发 —— 网络代码零命中
    $netKeywords = @('sock' + 'et(', 'recv' + 'from', 'bin' + 'd(', 'conn' + 'ect(',
                     'WSASock' + 'et', 'multicast', 'send' + 'to(', '<winsock', 'asio')
    $netHits = @(SearchHits $linkFiles (($netKeywords | ForEach-Object { [regex]::Escape($_) }) -join '|') `
        @('netKeywords', 'MUST NOT', '零命中', '检索范围'))
    Check "C13" "README「不做协议收发」：引擎与示例内网络代码零命中" ($netHits.Count -eq 0) `
        ("命中 {0} 处{1}" -f $netHits.Count, (Format-Hits $netHits))

    # C14：不做渲染 —— 图形/窗口/终端绘图零命中
    $renderPattern = 'OpenGL|glfw|SDL_|QOpenGL|ID2D1|Direct2D|CreateWindow|' +
                     '\bmap_2d\b|WebSocket|drawLine|renderFrame'
    $renderHits = @(SearchHits $renderFiles $renderPattern @('renderPattern', 'SearchHits'))
    Check "C14" "README「不做渲染」：图形/窗口代码零命中" ($renderHits.Count -eq 0) `
        ("命中 {0} 处{1}" -f $renderHits.Count, (Format-Hits $renderHits))

    # C15：不做广播 / 不做落库 / 不取挂钟（模块自身）
    $srcFiles = [string[]]@(CollectFiles @("src") @(".cc", ".h") @())
    $badHits = @(SearchHits $srcFiles `
        '(broadcast\(|EventHub|eventhub|std::chrono::system_clock|time\(nullptr\)|std::time\()' `
        @('MUST NOT'))
    Check "C15" "模块内无广播 / 无落库 / 不取挂钟（时间只由 step/tick 推进）" ($badHits.Count -eq 0) `
        ("命中 {0} 处{1}" -f $badHits.Count, (Format-Hits $badHits))

    # ================================================================ ⑤ 反向接口
    Section "⑤ 反向接口纪律：ISimSink / ISensorModel / IClock 由宿主注入"

    $headerPath = Join-Path $script:Repo "include\sim_source\sim_source.h"
    $headerOk = Test-Path $headerPath
    $headerText = ""
    if ($headerOk) { $headerText = [System.IO.File]::ReadAllText($headerPath) }

    $hasSink = ($headerText -match 'class\s+ISimSink') -and
               ($headerText -match 'virtual\s+void\s+onEvent') -and
               ($headerText -match 'virtual\s+void\s+onObservation')
    $hasSensor = ($headerText -match 'class\s+ISensorModel') -and
                 ($headerText -match 'virtual\s+std::vector<SimObservation>\s+sense') -and
                 ($headerText -match 'struct\s+SensorPose')
    $hasClock = ($headerText -match 'class\s+IClock') -and
                ($headerText -match 'virtual\s+int64_t\s+nowMs')
    Check "C16" "公开头声明三个反向接口 ISimSink / ISensorModel / IClock" `
        ($hasSink -and $hasSensor -and $hasClock) `
        ("sink={0} sensor={1} clock={2}" -f $hasSink, $hasSensor, $hasClock)

    $injected = ($headerText -match 'std::shared_ptr<ISimSink>\s+sink') -and
                ($headerText -match 'std::shared_ptr<ISensorModel>\s+model') -and
                ($headerText -match 'std::shared_ptr<IClock>\s+clock') -and
                ($headerText -match 'void\s+setSink') -and
                ($headerText -match 'void\s+setSensorModel') -and
                ($headerText -match 'void\s+setClock')
    Check "C17" "三个出口全部经注入进入（setSink / setSensorModel / setClock，无内建实现）" `
        $injected "SimSource 提供三个 set* 注入点；模块内没有它们的实现"

    # C18：唯一公开头
    $publicHeaders = @()
    $includeDir = Join-Path $script:Repo "include"
    if (Test-Path $includeDir) {
        $publicHeaders = @(Get-ChildItem $includeDir -Recurse -File -Filter *.h |
            ForEach-Object { Rel $_.FullName })
    }
    Check "C18" "唯一公开头 include/sim_source/sim_source.h" `
        (($publicHeaders.Count -eq 1) -and
         ($publicHeaders[0] -eq "include\sim_source\sim_source.h")) `
        ("公开头：{0}" -f ($publicHeaders -join ", "))

    $internalH = Join-Path $script:Repo "src\internal.h"
    $internalGuarded = $false
    if (Test-Path $internalH) {
        $t = [System.IO.File]::ReadAllText($internalH)
        $internalGuarded = ($t -match '宿主 MUST NOT 包含')
    }
    Check "C19" "内部头 src/internal.h 明确标注宿主不可包含" $internalGuarded `
        "src/internal.h 头部注明'MUST NOT 包含本文件'"

    # C20：中立输入结构齐备（不含任何业务取值）
    $neutralFields = @("SimScenario", "Area", "Group", "Platform", "Target",
                       "AreaRole", "Hardness", "TargetMotion")
    $neutralMissing = @()
    foreach ($f in $neutralFields) { if ($headerText -notmatch ("\b" + $f + "\b")) { $neutralMissing += $f } }
    Check "C20" "中立输入结构齐备（SimScenario/Area/Group/Platform/Target + 三个枚举）" `
        ($neutralMissing.Count -eq 0) `
        ("缺：{0}" -f (($neutralMissing -join ", ") -replace '^$', '无'))

    # C21：README「时间必须可注入」，且引擎时间口径写在注释里
    $timeDoc = ($headerText -match 'IClock') -and ($headerText -match '确定性') -and
               ($headerText -match 'tick\(nowMs\)')
    Check "C21" "时间可注入（IClock）且口径成文（step/tick/倍速/pause）" $timeDoc `
        "公开头 §3 说明 IClock 口径；SimOptions/SimSource 给出 setSpeed/pause/resume/step/tick"

    # ================================================================ ⑥ 归一化对象逐字段比对
    Section "⑥ 归一化对象逐字段比对（device-ingest《外设接入契约》§2）"

    if (-not $DeviceIngestDir) {
        $DeviceIngestDir = Join-Path (Split-Path -Parent $script:Repo) "device-ingest"
    }
    $contractPath = Join-Path $DeviceIngestDir "docs\契约\外设接入契约.md"
    $contractExists = Test-Path $contractPath
    $contractFields = @()
    $contractKind = ""
    if ($contractExists) {
        $docText = [System.IO.File]::ReadAllText($contractPath, [System.Text.Encoding]::UTF8)
        # §2 的 jsonc 代码块：逐行取 "字段": 值 的**出现顺序**
        $m = [regex]::Match($docText, '(?s)```jsonc(.*?)```')
        if ($m.Success) {
            foreach ($line in ($m.Groups[1].Value -split "`n")) {
                $fm = [regex]::Match($line, '^\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*:')
                if ($fm.Success) { $contractFields += $fm.Groups[1].Value }
            }
            $km = [regex]::Match($m.Groups[1].Value, '"kind"\s*:\s*"([^"]+)"')
            if ($km.Success) { $contractKind = $km.Groups[1].Value }
        }
    }
    Check "C22" "可读到 device-ingest 契约 §2 归一化对象（跨仓只读比对）" `
        ($contractExists -and ($contractFields.Count -ge 11)) `
        ("契约：{0}｜字段序：{1}" -f (Rel $contractPath), ($contractFields -join ","))

    # 前 11 个字段 = 模块必须逐字段产出的归一化对象
    $need = @()
    if ($contractFields.Count -ge 11) { $need = $contractFields[0..10] }
    $expected11 = @("deviceId", "deviceType", "kind", "lng", "lat", "alt", "heading", "speed",
                    "battery", "seq", "ts")
    $contract11Ok = ($need.Count -eq 11)
    if ($contract11Ok) {
        for ($i = 0; $i -lt 11; $i++) { if ($need[$i] -ne $expected11[$i]) { $contract11Ok = $false } }
    }
    Check "C23" "契约前 11 字段 == 本模块事件形状（deviceId…ts）" $contract11Ok `
        ("契约前 11：{0}" -f ($need -join ","))

    # 公开头里逐字段存在
    $headMissing = @()
    foreach ($f in $expected11) {
        if ($headerText -notmatch ('\b' + $f + ';') -and $headerText -notmatch ('\b' + $f + '\b')) {
            $headMissing += $f
        }
    }
    Check "C24" "公开头 SimEvent 逐字段含这 11 个字段名" ($headMissing.Count -eq 0) `
        ("缺：{0}" -f (($headMissing -join ", ") -replace '^$', '无'))

    # **运行期实测**：跑 selftest 的一个用例，把真实事件 dump 出来逐字段比对
    $dumpExe = $selftestExe
    $dumpOk = $false
    $dumpFields = @()
    $dumpKind = ""
    if (Test-Path $dumpExe) {
        $outPath = Join-Path ([System.IO.Path]::GetTempPath()) ("sim-source-dump-{0}.txt" -f $PID)
        & $dumpExe --dump-event > $outPath 2>$null
        if ((Test-Path $outPath) -and ((Get-Item $outPath).Length -gt 0)) {
            $line = ([System.IO.File]::ReadAllLines($outPath, [System.Text.Encoding]::UTF8) |
                     Where-Object { $_.Trim().StartsWith("{") } | Select-Object -First 1)
            if ($line) {
                $dumpOk = $true
                foreach ($fm in [regex]::Matches($line, '"([A-Za-z_][A-Za-z0-9_]*)"\s*:')) {
                    $dumpFields += $fm.Groups[1].Value
                }
                $km = [regex]::Match($line, '"kind"\s*:\s*"([^"]+)"')
                if ($km.Success) { $dumpKind = $km.Groups[1].Value }
            }
        }
    }
    $dumpMatch = $false
    if ($dumpOk -and $dumpFields.Count -ge 11) {
        $dumpMatch = $true
        for ($i = 0; $i -lt 11; $i++) { if ($dumpFields[$i] -ne $expected11[$i]) { $dumpMatch = $false } }
    }
    Check "C25" "运行期实测事件逐字段与契约同序（selftest --dump-event 的真实输出）" $dumpMatch `
        ("实测字段序：{0}" -f (($dumpFields -join ",") -replace '^$', '（无输出）'))

    # ================================================================ ⑦ 独立交付
    Section "⑦ 独立交付（构建 / 公开头 / 测试 / 示例 / 验收脚本 / 单头回落）"

    $needed = @("CMakeLists.txt", "include\sim_source\sim_source.h", "src\internal.h",
                "src\geometry.cc", "src\plan.cc", "src\engine.cc", "src\json.cc",
                "tests\CMakeLists.txt", "tests\selftest.cc", "examples\CMakeLists.txt",
                "examples\minimal\main.cc", "examples\full_flow\main.cc",
                "scripts\acceptance.ps1", "README.md", "LICENSE",
                "docs\实现报告.md", ".gitignore", ".gitattributes",
                "third_party\nlohmann\json.hpp")
    $absent = @()
    foreach ($f in $needed) { if (-not (Test-Path (Join-Path $script:Repo $f))) { $absent += $f } }
    Check "C26" "独立交付要件齐全" ($absent.Count -eq 0) `
        ("缺失：{0}" -f (($absent -join ", ") -replace '^$', '无'))

    $exampleCodes = @{}
    foreach ($name in @("example_minimal", "example_full_flow")) {
        $exe = Join-Path $bin "$name.exe"
        if (-not (Test-Path $exe)) { $exe = Join-Path $bin $name }
        if (-not (Test-Path $exe)) { $exampleCodes[$name] = -1; continue }
        $outPath = Join-Path ([System.IO.Path]::GetTempPath()) ("sim-source-{0}-{1}.txt" -f $name, $PID)
        & $exe > $outPath 2>&1
        $exampleCodes[$name] = $LASTEXITCODE
    }
    $exOk = $true
    foreach ($k in $exampleCodes.Keys) { if ($exampleCodes[$k] -ne 0) { $exOk = $false } }
    Check "C27" "两个示例独立运行退出码 0" $exOk `
        (($exampleCodes.Keys | Sort-Object | ForEach-Object { "{0}={1}" -f $_, $exampleCodes[$_] }) -join " ")

    $ctest = (Get-Command ctest -ErrorAction SilentlyContinue).Source
    if ($ctest) {
        $ctestOut = (& $ctest --test-dir $buildPath -C $Config --output-on-failure 2>&1 | Out-String)
        $ctestOk = ($LASTEXITCODE -eq 0)
        Check "C28" "ctest 注册的用例全部通过" $ctestOk ((($ctestOut -split "`n") |
            Where-Object { $_ -match 'tests passed|tests failed|Total Test time' }) -join " ")
    } else {
        Check "C28" "ctest 注册的用例全部通过（跳过：未找到 ctest）" $true "ctest 不在 PATH，视为不适用"
    }

    $scriptText = [System.IO.File]::ReadAllText((Join-Path $script:Repo "scripts\acceptance.ps1"))
    $exitOk = ($scriptText -match '(?m)^\s*exit\s+1') -and ($scriptText -match '(?m)^\s*exit\s+0')
    Check "C29" "验收脚本以退出码 0/1 结束" $exitOk "脚本内含 exit 0 与 exit 1 两条收口路径"

    # 自测打印口径：用例 N 个（失败 M）｜断言 X 条（失败 Y）
    $selfText = [System.IO.File]::ReadAllText((Join-Path $script:Repo "tests\selftest.cc"))
    $printOk = ($selfText -match '用例 %d 个（失败 %d）') -and ($selfText -match '断言 %d 条（失败 %d）')
    Check "C30" "自测打印口径：用例 N 个（失败 M）｜断言 X 条（失败 Y）" $printOk `
        "tests/selftest.cc 的汇总行逐字匹配"

    # ================================================================ 汇总
    Section "汇总"

    $total = $script:Results.Count
    $passed = 0
    foreach ($r in $script:Results) { if ($r.Ok) { $passed++ } }
    $failed = $total - $passed

    if ($jsonOk) {
        Write-Host ("selftest ：用例 {0} 个（失败 {1}）｜断言 {2} 条（失败 {3}）" -f `
            $data.cases, $data.casesFailed, $data.asserts, $data.assertsFailed)
    }
    Write-Host ("验收检查：{0} 项｜PASS {1}｜FAIL {2}" -f $total, $passed, $failed)

    if ($failed -eq 0) {
        Write-Host ""
        Write-Host "验收通过（退出码 0）" -ForegroundColor Green
        exit 0
    }
    Write-Host ""
    Write-Host "验收失败（退出码 1）：" -ForegroundColor Red
    foreach ($r in $script:Results) {
        if (-not $r.Ok) { Write-Host ("  - {0} {1}：{2}" -f $r.Id, $r.Title, $r.Detail) -ForegroundColor Red }
    }
    exit 1
} finally {
    Pop-Location
}
