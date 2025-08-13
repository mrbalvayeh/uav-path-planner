#include "DEMMap.h"
#include "cpl_conv.h"
#include "gdal_priv.h"
#include "ogr_spatialref.h"
#include <stdexcept>

DEMMap::DEMMap(const std::string &filename) {
    GDALAllRegister();
    loadDEM(filename);
}

DEMMap::~DEMMap() {
    if (elevationData_) CPLFree(elevationData_);
    if (dataset_) GDALClose(dataset_);
}

void DEMMap::loadDEM(const std::string &filename) {
    dataset_ = (void *) GDALOpen(filename.c_str(), GA_ReadOnly);
    if (!dataset_) throw std::runtime_error("Failed to open DEM file.");

    GDALDataset *ds = static_cast<GDALDataset *>(dataset_);
    GDALRasterBand *band = ds->GetRasterBand(1);
    width_ = band->GetXSize(); height_ = band->GetYSize();
    elevationData_ = (float *) CPLMalloc(sizeof(float) * width_ * height_);
    band->RasterIO(GF_Read, 0, 0, width_, height_, elevationData_, width_, height_, GDT_Float32, 0, 0);
    ds->GetGeoTransform(geoTransform_);

    OGRSpatialReference srs;
    char *wkt = const_cast<char *>(ds->GetProjectionRef());
    if (srs.importFromWkt(&wkt) != OGRERR_NONE) throw std::runtime_error("Failed to read projection");
    utmZone_ = srs.GetUTMZone();
    isNorthern_ = true;
}

double DEMMap::getElevation(double x, double y) const {
    int px = (x - geoTransform_[0]) / geoTransform_[1];
    int py = (y - geoTransform_[3]) / geoTransform_[5];
    if (px < 0 || py < 0 || px >= width_ || py >= height_) return -9999;
    return elevationData_[py * width_ + px];
}

bool DEMMap::isInBounds(double x, double y) const {
    int px = (x - geoTransform_[0]) / geoTransform_[1];
    int py = (y - geoTransform_[3]) / geoTransform_[5];
    return px >= 0 && py >= 0 && px < width_ && py < height_;
}

void DEMMap::latlonToUTM(double lat,double lon,double &x,double &y) const {
    OGRSpatialReference src, dst;
    src.SetWellKnownGeogCS("WGS84");
    src.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);     // <- add

    dst.SetUTM(utmZone_, isNorthern_);
    dst.SetWellKnownGeogCS("WGS84");
    dst.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);     // <- add

    auto *tr = OGRCreateCoordinateTransformation(&src,&dst);
    x = lon;  y = lat;
    if (!tr->Transform(1,&x,&y))
        throw std::runtime_error("latlonToUTM failed");
    OCTDestroyCoordinateTransformation(tr);
}

void DEMMap::utmToLatlon(double x, double y, double &lat, double &lon) const {
    OGRSpatialReference src, dst;
    src.SetUTM(utmZone_, isNorthern_); src.SetWellKnownGeogCS("WGS84");
    dst.SetWellKnownGeogCS("WGS84");
    auto *transform = OGRCreateCoordinateTransformation(&src, &dst);
    lon = x; lat = y;
    if (!transform->Transform(1, &lon, &lat)) throw std::runtime_error("utmToLatlon failed");
    OCTDestroyCoordinateTransformation(transform);
}
