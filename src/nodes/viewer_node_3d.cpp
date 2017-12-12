#include "qt/scenemodifier.h"

#include <ui_viewer_3d_main_widget.h>
#include "qt/viewer_3d_renderer.h"
#include "qt/viewer_3d_window.h"
#include "qt/viewer_3d_model.h"
#include "qt/viewer_3d_controller.h"

#include <QCommandLinkButton>
#include <QCheckBox>
#include <QObject>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cslibs_mapping_viewer_3d_node");
    QApplication app(argc, argv);

    cslibs_mapping::Viewer3dWindow window(&app);
    window.setup("test name");
    cslibs_mapping::Viewer3dModel model(&app);
    model.setup();
    cslibs_mapping::Viewer3dRenderer renderer(&app);
    renderer.setup(window.getView(),
                   &model);
    cslibs_mapping::Viewer3dController controller;
    controller.setup(&window, &model, &renderer);

    return app.exec();
}
