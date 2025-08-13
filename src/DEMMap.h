#ifndef DEMMAP_H
#define DEMMAP_H

#include <string>

class DEMMap {
public:
    explicit DEMMap(const std::string &filename);
    ~DEMMap();

    double getElevation(double x, double y) const;
    bool isInBounds(double x, double y) const;
    void latlonToUTM(double lat, double lon, double &x, double &y) const;
    void utmToLatlon(double x, double y, double &lat, double &lon) const;
    const double *getGeoTransform() const { return geoTransform_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    bool isAboveTerrain(double x, double y, double z, double minHAGL) const {
        return z >= getElevation(x, y) + minHAGL;
    }
private:
    void loadDEM(const std::string &filename);
    void *dataset_ = nullptr;
    float *elevationData_ = nullptr;
    double geoTransform_[6]{};
    int width_ = 0, height_ = 0;
    int utmZone_ = 39;
    bool isNorthern_ = true;
};

#endif
