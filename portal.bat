@echo off
setlocal enabledelayedexpansion

:: Arduino 빌드 폴더 기본 경로
set "arduinoBuildPath=C:\Users\사용자명\AppData\Local\arduino\sketches"
set "projectPath=타겟경로"

:: 가장 최신 폴더를 찾기 위한 변수 초기화
set "latestFolder="
set "latestTime=0"

:: 폴더 목록을 조회하며 최신 날짜의 폴더 찾기
for /d %%F in ("%arduinoBuildPath%\*") do (
    for %%G in ("%%F") do (
        set "folderTime=%%~tG"
        set "folderName=%%F"

        :: 폴더의 마지막 수정 시간이 가장 최신인지 확인
        if !folderTime! GTR !latestTime! (
            set "latestTime=!folderTime!"
            set "latestFolder=!folderName!"
        )
    )
)

:: 최신 폴더에서 바이너리 파일 복사
if not "!latestFolder!"=="" (
    echo ?? 가장 최근 생성된 폴더: !latestFolder!
    copy /Y "!latestFolder!\*.bin" "%projectPath%\"
    echo ? 최신 바이너리 파일이 프로젝트 폴더로 이동 완료!
) else (
    echo ? 최신 폴더를 찾을 수 없습니다. 컴파일을 먼저 실행하세요.
)

pause
