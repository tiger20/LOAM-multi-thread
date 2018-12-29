#pragma once
#include "Twist.h"
#include "nanoflann_pcl.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace loam
{

  /** \brief Implementation of the LOAM laser odometry component.
   *
   */
  class BasicLaserOdometry
  {
  public:
    explicit BasicLaserOdometry(float scanPeriod = 0.1, size_t maxIterations = 25);

    /** \brief Try to process buffered data. */
    void process();
    void updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans);

    pcl::PointCloud<pcl::PointXYZI>::Ptr& cornerPointsSharp()     { return _cornerPointsSharp; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cornerPointsLessSharp() { return _cornerPointsLessSharp; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& surfPointsFlat()        { return _surfPointsFlat; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& surfPointsLessFlat()    { return _surfPointsLessFlat; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloud() { return _laserCloud; }

    Twist const& transformSum() { return _transformSum; }
    Twist const& transform()    { return _transform;    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr const& lastCornerCloud () { return _lastCornerCloud ; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr const& lastSurfaceCloud() { return _lastSurfaceCloud; }

    void setScanPeriod(float val)     { _scanPeriod    = val; }
    void setMaxIterations(size_t val) { _maxIterations = val; }
    void setDeltaTAbort(float val)    { _deltaTAbort = val;   }
    void setDeltaRAbort(float val)    { _deltaRAbort = val;   }

    long frameCount()    const { return _frameCount;    }
    float scanPeriod()    const { return _scanPeriod;    }
    size_t maxIterations() const { return _maxIterations; }
    float deltaTAbort()   const { return _deltaTAbort;   }
    float deltaRAbort()   const { return _deltaRAbort;   }

    /** \brief Transform the given point cloud to the end of the sweep.
     *
     * @param cloud the point cloud to transform
     */
    size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  private:
    /** \brief Transform the given point to the start of the sweep.
     *
     * @param pi the point to transform
     * @param po the point instance for storing the result
     */
    void transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po);


    void pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                           const Angle& blx, const Angle& bly, const Angle& blz,
                           const Angle& alx, const Angle& aly, const Angle& alz,
                           Angle &acx, Angle &acy, Angle &acz);

    void accumulateRotation(Angle cx, Angle cy, Angle cz,
                            Angle lx, Angle ly, Angle lz,
                            Angle &ox, Angle &oy, Angle &oz);

  public:
    float _scanPeriod;       ///< time per scan
    long _frameCount;        ///< number of processed frames
    size_t _maxIterations;   ///< maximum number of iterations
    bool _systemInited;      ///< initialization flag

    float _deltaTAbort;     ///< optimization abort threshold for deltaT
    float _deltaRAbort;     ///< optimization abort threshold for deltaR

    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastCornerCloud;    ///< last corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastSurfaceCloud;   ///< last surface points cloud

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudOri;      ///< point selection
    pcl::PointCloud<pcl::PointXYZI>::Ptr _coeffSel;           ///< point selection coefficients

    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree;   ///< last corner cloud KD-tree
    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastSurfaceKDTree;  ///< last surface cloud KD-tree

    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsSharp;      ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsLessSharp;  ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsFlat;         ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsLessFlat;     ///< less flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;             ///< full resolution cloud

    std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer
    std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer

    std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer
    std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer
    std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer

    Twist _transform;     ///< optimized pose transformation
    Twist _transformSum;  ///< accumulated optimized pose transformation

    Angle _imuRollStart, _imuPitchStart, _imuYawStart;
    Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;

    Vector3 _imuShiftFromStart;
    Vector3 _imuVeloFromStart;
  };

} // end namespace loam
