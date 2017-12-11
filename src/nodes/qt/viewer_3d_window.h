#ifndef CSLIBS_MAPPING_VIEWER_3D_WINDOW_H
#define CSLIBS_MAPPING_VIEWER_3D_WINDOW_H

#include <QObject>

#include <ui_viewer_3d_main_widget.h>

namespace Qt3D {
    class QWindow;
}

namespace cslibs_mapping {
class Viewer3dWindow : public QObject
{
    Q_OBJECT

public:
    Viewer3dWindow(QObject *parent = nullptr);
    virtual ~Viewer3dWindow();

    void setup(const std::string &window_name);

    Ui::Viewer3dMainWidget *getUi();
    QWidget                *getViewContainer();
    Qt3D::QWindow          *getView();
    float                   getAspectRatio();

signals:
    void aspectRatioChanged();
    void mouseWheelChanged(const int dir);

private:
    Qt3D::QWindow          *view_;
    QWidget                *view_container_;

    Ui::Viewer3dMainWidget *ui_;

    bool eventFilter(QObject *object, QEvent *event) override;


};
}

#endif // CSLIBS_MAPPING_VIEWER_3D_WINDOW_H
