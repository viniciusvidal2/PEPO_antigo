/**
 * @file /include/programinha/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef programinha_QNODE_HPP_
#define programinha_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../msgs/imagem.pb.h"
#include "../../msgs/nuvem.pb.h"
#include "../../msgs/arquivos.pb.h"
#include "../../msgs/nvm.pb.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream.h"

#include <zmq.hpp>
#include <zmq_utils.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace ArquivosMsgProto;
using namespace ImagemMsgProto;
using namespace NuvemMsgProto;
using namespace NVMMsgProto;
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace zmq;

namespace programinha {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

Q_SIGNALS:
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
    ros::Publisher im_pub;
    ros::Publisher cl_proj_pub;
    ros::Publisher cl_pub;
};

}  // namespace programinha

#endif /* programinha_QNODE_HPP_ */
