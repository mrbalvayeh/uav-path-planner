import pandas as pd
import matplotlib.pyplot as plt
import rasterio
from rasterio.plot import show
from pyproj import Transformer, Geod
from matplotlib.patches import Polygon as MplPolygon

# Load the DEM
dem_path = 'DEM_UTM.tif'
with rasterio.open(dem_path) as dem:
    fig, ax = plt.subplots(figsize=(10, 10))
    show(dem, ax=ax, cmap='gray')

    # === Load waypoints ===
    df = pd.read_csv('path_waypoints.csv')
    lon = df.iloc[:, 0].astype(float).values
    lat = df.iloc[:, 1].astype(float).values

    # === Reproject waypoints ===
    transformer = Transformer.from_crs("EPSG:4326", dem.crs, always_xy=True)
    x, y = transformer.transform(lon, lat)
    ax.plot(x, y, 'ro-', label='Waypoints')

    # === Straight blue line from first to last waypoint ===
    x_start, y_start = x[0], y[0]
    x_end, y_end = x[-1], y[-1]
    ax.plot([x_start, x_end], [y_start, y_end], color='blue', linewidth=2, linestyle='--', label='Direct Path')

    # === Avoid Zones: list of (lat, lon, radius_km) ===
    avoid_zones = [
        (26.9684, 54.393, 20),
        (28.2329, 54.0702, 40),
        (29.919, 53.4707, 30),
        (29.234, 55.638, 30)
    ]

    geod = Geod(ellps="WGS84")

    # Draw each avoid zone as a circle
    for idx, (lat_c, lon_c, radius_km) in enumerate(avoid_zones):
        circle_points = []
        for azimuth in range(360):
            lon2, lat2, _ = geod.fwd(lon_c, lat_c, azimuth, radius_km * 1000)
            circle_points.append((lon2, lat2))
        circle_lons, circle_lats = zip(*circle_points)
        circle_x, circle_y = transformer.transform(circle_lons, circle_lats)

        polygon = MplPolygon(
            list(zip(circle_x, circle_y)),
            closed=True,
            edgecolor='red',
            facecolor='red',
            alpha=0.3,
            linewidth=1.5,
            label='Avoid Zone' if idx == 0 else None
        )
        ax.add_patch(polygon)

    ax.legend()
    ax.set_title('Waypoints, Avoid Zones, and Direct Path over DEM')
    ax.set_xlabel('Easting')
    ax.set_ylabel('Northing')

plt.tight_layout()
plt.show()
