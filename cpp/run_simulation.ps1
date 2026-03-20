# ---------- 1. Compile the C++ program ----------
Write-Host "--- Compiling C++ Simulation ---" -ForegroundColor Cyan

g++ -O2 -std=c++20 main.cpp -o main.exe

# Check if compilation succeeded
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Compilation failed. Exiting script." -ForegroundColor Red
    exit $LASTEXITCODE
}

# ---------- 2. Run the C++ simulation ----------
Write-Host "`n--- Running C++ Network Simulation ---" -ForegroundColor Cyan

.\main.exe

# Check if the program executed correctly
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: main.exe encountered a problem. Exiting script." -ForegroundColor Red
    exit $LASTEXITCODE
}

# ---------- 3. Generate visualization ----------
Write-Host "`n--- Generating Visualization ---" -ForegroundColor Cyan

# Path to virtual environment Python
$venvPython = ".\..\venv\Scripts\python.exe"

if (-Not (Test-Path $venvPython)) {
    Write-Host "Error: Virtual environment python not found at $venvPython." -ForegroundColor Red
    Write-Host "Please ensure your 'venv' folder is set up correctly." -ForegroundColor Yellow
    exit 1
}

# Run visualizer
& $venvPython visualiser.py

Write-Host "`n--- Pipeline Complete ---" -ForegroundColor Green