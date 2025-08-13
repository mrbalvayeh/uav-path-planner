# UAV Path Planner – Terrain & Constraint Aware

A **C++ UAV path planning system** for fixed-wing aircraft that uses **OMPL** (Open Motion Planning Library) and **GDAL** to generate realistic, constraint-aware flight paths over **Digital Elevation Models (DEM)**.  

The system considers terrain, slope, no-fly zones, turning radius limits, and dynamic altitude requirements to produce safe, flyable paths. Includes a Python visualization script for inspection.

---

## ✨ Features
- **Fixed-wing flight model** with Dubins curves
- **Slope-dependent minimum altitude (HAGL)**
- **Circular avoid zones** (GPS jammers, no-fly zones, LOS restrictions)
- **Terrain ceiling enforcement**
- **Multi-leg path planning** with automatic bypass waypoints
- **Custom optimization objective** factoring:
  - Path length
  - Elevation gain
  - Terrain slope
  - Heading alignment
- **Terrain-aware shortcutting** for smoother paths
- **CSV waypoint export** with:
  - Latitude / Longitude
  - Terrain elevation
  - Flight altitude
  - HAGL
  - Heading
- **Python visualization** of path and avoid zones over DEM

---

## 📂 Project Structure
```
├── CMakeLists.txt              # Build config for CLion / CMake
├── main.cpp                    # Main path planner
├── src/
│   ├── DEMMap.cpp / DEMMap.h   # DEM loading, elevation lookup, coordinate transforms
│   ├── ConstraintChecker.cpp/.h# Flight safety & slope rules
│   ├── WaypointPath.h          # Waypoint data structure
├── plot_waypoints_on_dem.py    # Python visualization
└── DEM_UTM.tif                 # Sample DEM (ignored in GitHub)
```

---

## 🚀 Build & Run

### 1. **Dependencies**
- **C++20**
- **CMake ≥ 3.10**
- [OMPL](https://ompl.kavrakilab.org/)  
- [GDAL](https://gdal.org/)  

Install on Debian/Ubuntu:
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libompl-dev libgdal-dev
```

---

### 2. **Build**
```bash
git clone https://github.com/YOUR-USERNAME/uav-path-planner.git
cd uav-path-planner
mkdir build && cd build
cmake ..
make -j$(nproc)
```

---

### 3. **Run**
```bash
./path_planner
```
The program will generate `path_waypoints.csv` containing the planned waypoints.

---

## 📊 Visualization
Use the provided Python script to plot the planned route over the DEM.

```bash
pip install pandas matplotlib rasterio pyproj
python plot_waypoints_on_dem.py
```

---

## ⚠️ DEM File
- The sample DEM (`DEM_UTM.tif`) is **not** included in the repo (too large).  
- Place your DEM file locally and update its path in `main.cpp`:
```cpp
demMap = std::make_shared<DEMMap>("/path/to/DEM_UTM.tif");
```

---

## 📜 License
MIT License – feel free to use, modify, and share.
