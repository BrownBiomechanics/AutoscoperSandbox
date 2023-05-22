/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

/*
Note: This file and its associated CMakeList.txt file must be built within the examples
folder in iMSTK to run. It is used as a testbed for the multiple collision detection 
methods within iMSTK for rigid bone meshes from autoscoper. The current setup
uses a SDF representation of one of the bones to generate collision data and 
handle collisions with the point set defined by the other bone. 
*/

#include "imstkCamera.h"
#include "imstkDeviceManager.h"
#include "imstkDeviceManagerFactory.h"
#include "imstkDirectionalLight.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLevelSetCH.h"
#include "imstkLevelSetModel.h"
#include "imstkMeshIO.h"
#include "imstkMouseDeviceClient.h"
#include "imstkMouseSceneControl.h"
#include "imstkObjectControllerGhost.h"
#include "imstkPbdModel.h"
#include "imstkPbdModelConfig.h"
#include "imstkPbdObject.h"
#include "imstkPbdObjectCollision.h"
#include "imstkPbdObjectController.h"
#include "imstkRigidObjectLevelSetCollision.h"
#include "imstkScene.h"
#include "imstkSceneManager.h"
#include "imstkSimulationManager.h"
#include "imstkSimulationUtils.h"
#include "imstkSurfaceMesh.h"
#include "imstkVisualModel.h"
#include "imstkVolumeRenderMaterial.h"
#include "imstkVTKViewer.h"

#include "imstkSurfaceMeshDistanceTransform.h"

#ifndef iMSTK_USE_HAPTICS
#include "imstkDummyClient.h"
#include "imstkMouseDeviceClient.h"
#endif

using namespace imstk;

std::shared_ptr<PbdObject>
makeRigidObj(const std::string& name, std::shared_ptr<PbdModel> pbdModel)
{
    auto rigidObj = std::make_shared<PbdObject>(name);
    
    
    
    auto boneMesh = MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "Bone/Bonemc3.stl");
    auto center = boneMesh->getCenter();
    boneMesh->translate(-center, Geometry::TransformType::ApplyToData);

   /* Vec3d shift = Vec3d(0.0, 40, 0.0);
    boneMesh->translate(shift, Geometry::TransformType::ApplyToData);*/

    rigidObj->setDynamicalModel(pbdModel);
    rigidObj->getPbdBody()->setRigid(
        Vec3d(0.0, 0.00, 0.0),            // Position
        0.1,                              // Mass
        Quatd::Identity(),                // Orientation
        Mat3d::Identity() * 100000.0);    // Inertia
 

    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::Surface);
    material->setShadingModel(RenderMaterial::ShadingModel::PBR);
    material->setMetalness(0.9);
    material->setRoughness(0.4);
    material->setDiffuseColor(Color(0.7, 0.7, 0.7));

    // Create the object
    rigidObj->setVisualGeometry(boneMesh);
    rigidObj->setPhysicsGeometry(boneMesh);
    rigidObj->setCollidingGeometry(boneMesh);
    rigidObj->getVisualModel(0)->setRenderMaterial(material);


    // Add a component for controlling via another device
    auto controller = rigidObj->addComponent<PbdObjectController>();
    controller->setControlledObject(rigidObj);
    // controller->setDevice(deviceClient);
    controller->setTranslationOffset(Vec3d(0.0, 0.0, 0.0));
    controller->setLinearKs(100.0);
    controller->setAngularKs(10000000.0);
    controller->setTranslationScaling(100.0);
    controller->setForceScaling(0.005);
    controller->setSmoothingKernelSize(10);
    controller->setUseForceSmoothening(true);
    controller->setUseCritDamping(true);

    // Add extra component to tool for the ghost
    auto controllerGhost = rigidObj->addComponent<ObjectControllerGhost>();
    controllerGhost->setUseForceFade(true);
    controllerGhost->setController(controller);

    return rigidObj;
}


std::shared_ptr<CollidingObject>
makeCDBone(const std::string& name)
{

    // Create the colliding bone
    auto boneObj = std::make_shared<CollidingObject>("CDBone");

    auto boneMesh = MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "Bone/Bonemc3.stl");
    auto center = boneMesh->getCenter();
    boneMesh->translate(-center, Geometry::TransformType::ApplyToData);


    boneMesh->rotate(Vec3d(0.0, 1.0, 0.0), 3.14 / 2, Geometry::TransformType::ApplyToData);

    Vec3d shift = Vec3d(0.0, -25, -20.0);
    boneMesh->translate(shift, Geometry::TransformType::ApplyToData);

    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::Surface);


    auto computeSdf = std::make_shared<SurfaceMeshDistanceTransform>();
    computeSdf->setInputMesh(boneMesh);
    computeSdf->setDimensions(150, 150, 150);
    computeSdf->update();

    

    // Create the object
    boneObj->setVisualGeometry(boneMesh);
    // boneObj->setCollidingGeometry(boneMesh);
    boneObj->setCollidingGeometry(std::make_shared<SignedDistanceField>(computeSdf->getOutputImage()));
    boneObj->getVisualModel(0)->setRenderMaterial(material);

    return boneObj;
}



///
/// \brief This example demonstrates cutting a femur bone with a tool
/// Some of the example parameters may need to be tweaked for differing
/// systems
///
int
main()
{
    // Setup logger (write to file and stdout)
    Logger::startLogger();

    auto scene = std::make_shared<Scene>("FemurCut");


    // Setup
    auto pbdModel = std::make_shared<PbdModel>();

    pbdModel->getConfig()->m_dt = 0.001;
    pbdModel->getConfig()->m_gravity = Vec3d::Zero();
    

    std::shared_ptr<PbdObject> rbdObj = makeRigidObj("ToolBone", pbdModel);
    scene->addSceneObject(rbdObj);

    std::shared_ptr<CollidingObject> cdBone = makeCDBone("CDBone");
    scene->addSceneObject(cdBone);

    auto collision = std::make_shared<PbdObjectCollision>(rbdObj, cdBone, "ImplicitGeometryToPointSetCD");
    //auto collision = std::make_shared<PbdObjectCollision>(rbdObj, cdBone);
    //collision->setRigidBodyCompliance(0.000001);
    scene->addSceneObject(collision);


    // Light
    auto light = std::make_shared<DirectionalLight>();
    light->setDirection(Vec3d(0.0, -8.0, -5.0));
    light->setIntensity(1.0);
    scene->addLight("light", light);

    // Adjust camera
    scene->getActiveCamera()->setFocalPoint(0.25, 0.83, 1.58);
    scene->getActiveCamera()->setPosition(0.0, 100, 140);
    scene->getActiveCamera()->setViewUp(0.0, 1.0, 0.0);

    {
        auto viewer = std::make_shared<VTKViewer>();
        viewer->setVtkLoggerMode(VTKViewer::VTKLoggerMode::MUTE);
        viewer->setActiveScene(scene);

        // Add a module to run the scene
        auto sceneManager = std::make_shared<SceneManager>();
        sceneManager->setActiveScene(scene);

        auto driver = std::make_shared<SimulationManager>();
        driver->addModule(viewer);
        driver->addModule(sceneManager);
        driver->setDesiredDt(0.001); // Exactly 1000ups

#ifdef iMSTK_USE_HAPTICS
        // Setup default haptics manager
        std::shared_ptr<DeviceManager> hapticManager = DeviceManagerFactory::makeDeviceManager();
        driver->addModule(hapticManager);
        std::shared_ptr<DeviceClient> deviceClient = hapticManager->makeDeviceClient();
#else
        auto deviceClient = std::make_shared<DummyClient>();
        connect<Event>(sceneManager, &SceneManager::postUpdate, [&](Event*)
            {
                const Vec2d mousePos = viewer->getMouseDevice()->getPos();
                const Vec3d worldPos = Vec3d(mousePos[0] * 0.5 - 0.5, mousePos[1] * 0.2 + 0.1, -0.025);
                deviceClient->setPosition(worldPos);
            });
#endif

        auto controller = rbdObj->getComponent<PbdObjectController>();
        controller->setDevice(deviceClient);

        /*std::shared_ptr<ObjectControllerGhost> ghostObj = rbdObj->addComponent<ObjectControllerGhost>();
        ghostObj->setController(controller);*/

        connect<Event>(sceneManager, &SceneManager::preUpdate,
            [&](Event*)
            {
                rbdObj->getPbdModel()->getConfig()->m_dt = sceneManager->getDt();
        });

        // Add default mouse and keyboard controls to the viewer
        std::shared_ptr<Entity> mouseAndKeyControls =
            SimulationUtils::createDefaultSceneControl(driver);
        scene->addSceneObject(mouseAndKeyControls);

        driver->start();
    }

    return 0;
}
