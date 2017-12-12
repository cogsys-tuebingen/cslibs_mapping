#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkLight.h>

#include <map>
#include <cslibs_mapping/Distributions3d.h>



int main(int, char *[])
{
    // The following lines create a sphere represented by polygons.
    //
    vtkSmartPointer<vtkSphereSource> sphere =
            vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetThetaResolution(100);
    sphere->SetPhiResolution(50);

    // The mapper is responsible for pushing the geometry into the graphics
    // library. It may also do color mapping, if scalars or other attributes
    // are defined.
    //
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphere->GetOutputPort());

    // The actor is a grouping mechanism: besides the geometry (mapper), it
    // also has a property, transformation matrix, and/or texture map.
    // In this example we create eight different spheres (two rows of four
    // spheres) and set the specular lighting coefficients. A little ambient
    // is turned on so the sphere is not completely black on the back side.
    //


    // Create the graphics structure. The renderer renders into the
    // render window. The render window interactor captures mouse events
    // and will perform appropriate camera or actor manipulation
    // depending on the nature of the events.
    //
    vtkSmartPointer<vtkRenderer> ren1 =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renWin =
            vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(ren1);
    vtkSmartPointer<vtkRenderWindowInteractor> iren =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(renWin);

    // Add the actors to the renderer, set the background and size.
    //
    for(std::size_t i = 0 ; i < 5000000 ; ++i) {
        vtkSmartPointer<vtkActor> sphere1 =
                vtkSmartPointer<vtkActor>::New();
        sphere1->SetMapper(sphereMapper);
        sphere1->GetProperty()->SetColor(1,0,0);
        sphere1->GetProperty()->SetAmbient(0.3);
        sphere1->GetProperty()->SetDiffuse(0.0);
        sphere1->GetProperty()->SetSpecular(1.0);
        sphere1->GetProperty()->SetSpecularPower(5.0);
        sphere1->AddPosition(i/100000.0,0,0);
        ren1->AddActor(sphere1);
    }

    ren1->SetBackground(0.1, 0.2, 0.4);
    renWin->SetSize(640, 480);

    // Set up the lighting.
    //
    vtkSmartPointer<vtkLight> light =
            vtkSmartPointer<vtkLight>::New();
    light->SetFocalPoint(1.875,0.6125,0);
    light->SetPosition(0.875,1.6125,1);
    ren1->AddLight(light);

    // We want to eliminate perspective effects on the apparent lighting.
    // Parallel camera projection will be used. To zoom in parallel projection
    // mode, the ParallelScale is set.
    //
    ren1->GetActiveCamera()->SetFocalPoint(0,0,0);
    ren1->GetActiveCamera()->SetPosition(0,0,1);
    ren1->GetActiveCamera()->SetViewUp(0,1,0);
    ren1->GetActiveCamera()->ParallelProjectionOn();
    ren1->ResetCamera();
    ren1->GetActiveCamera()->SetParallelScale(1.5);

    // This starts the event loop and invokes an initial render.
    //
    iren->Initialize();
    iren->Start();

    return EXIT_SUCCESS;
}
