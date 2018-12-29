#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pangolin/pangolin.h>

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <queue>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <mutex>

//#include "loam_velodyne/BasicScanRegistration.h"
#include "loam_velodyne/MultiScanRegistration.h"
#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/TransformMaintenance.h"

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
int nFRAMES = 849;
std::mutex m_buf;
std::mutex m_odom2map,m_transMain;
queue<pcl::PointCloud<pcl::PointXYZI>> fullRes_buf;
queue<pcl::PointCloud<pcl::PointXYZI>> pointsSharp_buf;
queue<pcl::PointCloud<pcl::PointXYZI>> lessSharp_buf;
queue<pcl::PointCloud<pcl::PointXYZI>> pointsFlat_buf;
queue<pcl::PointCloud<pcl::PointXYZI>> lessFlat_buf;

queue<pcl::PointCloud<pcl::PointXYZI>> fullRes_odom2map;
queue<pcl::PointCloud<pcl::PointXYZI>> corner_odom2map;
queue<pcl::PointCloud<pcl::PointXYZI>> surf_odom2map;
queue<loam::Twist> trans_odom2map;

loam::MultiScanRegistration multiScan;
loam::LaserOdometry laserOdom;
loam::LaserMapping laserMap;
loam::TransformMaintenance transMain;
//double time_stamp = 0.5;

int MaxQueueLength = 3;

std::condition_variable con;

vector<loam::Twist> Trajectory;
vector<Eigen::Vector3d> Trajectory2;

pcl::PointCloud<pcl::PointXYZI>::Ptr pMap;
pcl::PointCloud<pcl::PointXYZI> laserReg;
int getRingForAngle(float angle)
{
    float _factor = (16.0 - 1.0) / 30.0;
    return int(((angle * 180.0 / M_PI) + 15) * _factor + 0.5);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
   // --------------------------------------------
   // -----Open 3D viewer and add point cloud-----
   // --------------------------------------------
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> displayFeatures(pcl::PointCloud<pcl::PointXYZI>::ConstPtr original, pcl::PointCloud<pcl::PointXYZI>::ConstPtr c1, pcl::PointCloud<pcl::PointXYZI>::ConstPtr c2)
{
    //     // --------------------------------------------
//     // -----Open 3D viewer and add point cloud-----
//     // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB> orig;
    pcl::PointCloud<pcl::PointXYZRGB> corner;
    pcl::PointCloud<pcl::PointXYZRGB> flat;
    for(int i = 0; i < original->points.size(); i++)
    {
        pcl::PointXYZRGB point;
        point.x = original->points[i].x;
        point.y = original->points[i].y;
        point.z = original->points[i].z;

        uint32_t rgb = (static_cast<uint32_t>(100) << 16 |
                        static_cast<uint32_t>(100) << 8 | static_cast<uint32_t>(100));
        point.rgb = rgb;
        orig.points.push_back(point);
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(orig.makeShared());
    viewer->addPointCloud<pcl::PointXYZRGB> (orig.makeShared(), color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    for(int i = 0; i < c1->points.size(); i++)
    {
        pcl::PointXYZRGB point;
        point.x = c1->points[i].x;
        point.y = c1->points[i].y;
        point.z = c1->points[i].z;

        uint32_t rgb = (static_cast<uint32_t>(255) << 16 |
                        static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));
        point.rgb = rgb;
        corner.points.push_back(point);
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color1(corner.makeShared());
    viewer->addPointCloud<pcl::PointXYZRGB> (corner.makeShared(), color1, "corner");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "corner");

    for(int i = 0; i < c2->points.size(); i++)
    {
        pcl::PointXYZRGB point;
        point.x = c2->points[i].x;
        point.y = c2->points[i].y;
        point.z = c2->points[i].z;

        uint32_t rgb = (static_cast<uint32_t>(125) << 16 |
                        static_cast<uint32_t>(125) << 8 | static_cast<uint32_t>(0));
        point.rgb = rgb;
        flat.points.push_back(point);
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color2(flat.makeShared());
    viewer->addPointCloud<pcl::PointXYZRGB> (flat.makeShared(), color2, "flat");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "flat");

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}
vector<string> loadData()
{
    std::vector<string> v;
    for(int i = 0; i < nFRAMES; i++)
    {
        stringstream sstr;
        sstr<<"/Users/walker/Downloads/nsh_indoor_outdoor_pcd/"<<i<<".pcd";
        v.push_back(sstr.str());
    }
    return v;
}
void Draw() 
{
    float fx = 277.34;
    float fy = 291.402;
    float cx = 312.234;
    float cy = 239.777;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 10, -10, 0, 0, 0, 0.0, 1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

   
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        
        glLineWidth(2);
        if(Trajectory2.size() > 1)
        {
            for(int i = 0;i < Trajectory2.size() - 1;i++)
            {
                glColor3f(0, 1, 0);
                glBegin(GL_LINES);
                auto p1 = Trajectory2[i], p2 = Trajectory2[i + 1];
                glVertex3d(p1[0], p1[1], p1[2]);
                glVertex3d(p2[0], p2[1], p2[2]);
                glEnd();
            }
        }


        if(pMap->points.size() > 2)
        {
            glPointSize(1);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < pMap->points.size(); i++) 
            {
                glColor3f(0.6, 0.6, 0.6);
                glVertex3d(pMap->points[i].x, 
                    pMap->points[i].y, 
                    pMap->points[i].z);
            }
            glEnd();
        }
        
        if(laserReg.points.size() > 2)
        {
            glPointSize(1);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < laserReg.points.size(); i++) 
            {
                glColor3f(1, 0, 0);
                glVertex3d(laserReg.points[i].x, 
                    laserReg.points[i].y, 
                    laserReg.points[i].z);
            }
            glEnd();
        }

        pangolin::FinishFrame();
        
        usleep(5000);   // sleep 5 ms
    }
}

int main (int argc, char** argv)
{
	//loam::BasicScanRegistration res;
	//res.nScans = 16;
    
    vector<string> files = loadData();

    // for(auto& f:files)
    //     cout<<f<<endl;
    std::thread registration([&](){
        for(int i = 0; i < nFRAMES; i++)
        {
            m_buf.lock();

            //cout<<i<<endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

            if (pcl::io::loadPCDFile<pcl::PointXYZ> (files[i], *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file \n");
                return (-1);
            }

            //std::chrono::steady_clock::time_point  now = std::chrono::steady_clock::now();
            multiScan.handleCloud(*cloud);
            fullRes_buf.emplace(std::move(multiScan._laserCloud));
            pointsSharp_buf.emplace(std::move(multiScan._cornerPointsSharp));
            lessSharp_buf.emplace(std::move(multiScan._cornerPointsLessSharp));
            pointsFlat_buf.emplace(std::move(multiScan._surfacePointsFlat));
            lessFlat_buf.emplace(std::move(multiScan._surfacePointsLessFlat));

            con.notify_one();
            //cout<<"multiscan._lasercloud: "<<multiScan._laserCloud.points.size()<<endl;
            // auto t2 = std::chrono::steady_clock::now();
            // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - now);
            // std::cout << "Feature extration time: " << time_span.count() << " seconds."<<endl;
            m_buf.unlock();
        }
    });
    

    std::thread odometry([&](){
        while(true)
        {

            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk);
            //std::chrono::steady_clock::time_point  now = std::chrono::steady_clock::now();
            if((!fullRes_buf.empty()) && 
               (!pointsSharp_buf.empty()) && 
               (!lessSharp_buf.empty()) && 
               (!pointsFlat_buf.empty()) && 
               (!lessFlat_buf.empty()))
            {
                pcl::PointCloud<pcl::PointXYZI> _fullRes = std::move(fullRes_buf.front());
                fullRes_buf.pop();
                pcl::PointCloud<pcl::PointXYZI> _cornerCloud = std::move(pointsSharp_buf.front());
                pointsSharp_buf.pop();
                pcl::PointCloud<pcl::PointXYZI> _lessSharp = std::move(lessSharp_buf.front());
                lessSharp_buf.pop();
                pcl::PointCloud<pcl::PointXYZI> _cloudFlat = std::move(pointsFlat_buf.front());
                pointsFlat_buf.pop();
                pcl::PointCloud<pcl::PointXYZI> _lessFlat = std::move(lessFlat_buf.front());
                lessFlat_buf.pop();
                laserOdom.readCloud(_fullRes, _cornerCloud, _lessSharp, _cloudFlat, _lessFlat);

            }
            else
                continue;
            lk.unlock();
            std::chrono::steady_clock::time_point  now = std::chrono::steady_clock::now();
            laserOdom.process();
            m_transMain.lock();
            transMain.handleLaserOdom(laserOdom.transformSum());
            Trajectory2.push_back(Eigen::Vector3d(transMain.transformMapped()[3],
                                                transMain.transformMapped()[4],
                                                transMain.transformMapped()[5]));
            m_transMain.unlock();
            m_odom2map.lock();
            if(laserOdom._ioRatio < 2 || laserOdom.frameCount() % laserOdom._ioRatio == 1 )
            {
                while(fullRes_odom2map.size() > MaxQueueLength)
                {
                    fullRes_odom2map.pop();
                    corner_odom2map.pop();
                    surf_odom2map.pop();
                    trans_odom2map.pop();
                }
                fullRes_odom2map.emplace(std::move(*(laserOdom.laserCloud())));
                corner_odom2map.emplace(std::move(*(laserOdom.lastCornerCloud())));
                surf_odom2map.emplace(std::move(*(laserOdom.lastSurfaceCloud())));
                trans_odom2map.push(laserOdom._transformSum);
            }
            m_odom2map.unlock();

            auto t2 = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - now);
            std::cout << "odometry process time: " << time_span.count() << " seconds."<<endl;
            //cout<<laserOdom._transformSum.pos.transpose()<<endl;
        }
    });
   
    pMap = laserMap._laserCloudSurroundDS;
    
    std::thread mapping([&](){
        while(true)
        {
            if((!fullRes_odom2map.empty()) && 
               (!corner_odom2map.empty()) && 
               (!surf_odom2map.empty()) &&
               (!trans_odom2map.empty()))
            {
                m_odom2map.lock();
                pcl::PointCloud<pcl::PointXYZI> _fullRes = std::move(fullRes_odom2map.front());
                fullRes_odom2map.pop();
                pcl::PointCloud<pcl::PointXYZI> _cornerCloud = std::move(corner_odom2map.front());
                corner_odom2map.pop();
                pcl::PointCloud<pcl::PointXYZI> _surfCloud = std::move(surf_odom2map.front());
                surf_odom2map.pop();
                loam::Twist trans = trans_odom2map.front();
                trans_odom2map.pop();
                m_odom2map.unlock();
                laserMap.readCloudAndOdom(_fullRes, _cornerCloud, _surfCloud, trans);
            }
            else
                continue;
            std::chrono::steady_clock::time_point  now = std::chrono::steady_clock::now();
            laserMap.process();
            laserReg = std::move(*(laserMap._laserCloudFullRes));
            m_transMain.lock();
            transMain.handleOdomAftMapped(laserMap.transformAftMapped(), laserMap.transformBefMapped());
            m_transMain.unlock();
            auto t2 = std::chrono::steady_clock::now();
            //Trajectory.push_back(laserMap.transformAftMapped());
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - now);
            std::cout << "mapping process time: " << time_span.count() << " seconds."<<endl;
        }
   });

    Draw();
    registration.join();
    odometry.join();
    mapping.join();
    return (0);
}
