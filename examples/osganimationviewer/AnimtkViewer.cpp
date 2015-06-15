/*  -*-c++-*-
 *  Copyright (C) 2008 Cedric Pinson <mornifle@plopbyte.net>
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
 *
 * Authors:
 * Cedric Pinson <mornifle@plopbyte.net>
 * jeremy Moles <jeremy@emperorlinux.com>
*/

#include "AnimtkViewerKeyHandler"
#include "AnimtkViewerGUI"

#include <iostream>
#include <osg/io_utils>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osgDB/FileNameUtils>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgWidget/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/Bone>
#include <osg/NodeVisitor>
#include <osgAnimation/RigGeometry>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/ref_ptr>
#include <limits>
#include <osg/Array>
#include <osgUtil/Optimizer>
#include <osgUtil/UpdateVisitor>
#include <osgAnimation/UpdateBone>

const int WIDTH  = 1440;
const int HEIGHT = 900;

osg::Geometry * createBox(const osg::BoundingBox &bb, const osg::Matrix &transform, osg::Vec4 color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)) {

    osg::Geometry *cube = new osg::Geometry;

    osg::Vec3 center = bb.center();
    double halfLenghtX = (bb._max.x() - bb._min.x()) * 0.5;
    double halfLenghtY = (bb._max.y() - bb._min.y()) * 0.5;
    double halfLenghtZ = (bb._max.z() - bb._min.z()) * 0.5;

    osg::Vec3Array *cubeVertices = new osg::Vec3Array;
    cubeVertices->push_back(osg::Vec3(center.x() - halfLenghtX, center.y() + halfLenghtY, center.z() + halfLenghtZ) * transform);
    cubeVertices->push_back(osg::Vec3(center.x() - halfLenghtX, center.y() + halfLenghtY, center.z() - halfLenghtZ) * transform);
    cubeVertices->push_back(osg::Vec3(center.x() - halfLenghtX, center.y() - halfLenghtY, center.z() - halfLenghtZ) * transform);
    cubeVertices->push_back(osg::Vec3(center.x() - halfLenghtX, center.y() - halfLenghtY, center.z() + halfLenghtZ) * transform);

    cubeVertices->push_back(osg::Vec3(center.x() + halfLenghtX, center.y() + halfLenghtY, center.z() + halfLenghtZ) * transform);
    cubeVertices->push_back(osg::Vec3(center.x() + halfLenghtX, center.y() + halfLenghtY, center.z() - halfLenghtZ) * transform);
    cubeVertices->push_back(osg::Vec3(center.x() + halfLenghtX, center.y() - halfLenghtY, center.z() - halfLenghtZ) * transform);
    cubeVertices->push_back(osg::Vec3(center.x() + halfLenghtX, center.y() - halfLenghtY, center.z() + halfLenghtZ) * transform);

    cube->setVertexArray(cubeVertices);

    osg::DrawElementsUInt* up =
            new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    up->push_back(0);
    up->push_back(1);
    up->push_back(5);
    up->push_back(4);
    cube->addPrimitiveSet(up);

    osg::DrawElementsUInt* down =
            new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    down->push_back(3);
    down->push_back(2);
    down->push_back(6);
    down->push_back(7);
    cube->addPrimitiveSet(down);

    osg::DrawElementsUInt* left =
            new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    left->push_back(0);
    left->push_back(1);
    left->push_back(2);
    left->push_back(3);
    cube->addPrimitiveSet(left);

    osg::DrawElementsUInt* right =
            new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    right->push_back(4);
    right->push_back(5);
    right->push_back(6);
    right->push_back(7);
    cube->addPrimitiveSet(right);

    osg::DrawElementsUInt* front =
            new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    front->push_back(0);
    front->push_back(4);
    front->push_back(7);
    front->push_back(3);
    cube->addPrimitiveSet(front);

    osg::DrawElementsUInt* back =
            new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    back->push_back(1);
    back->push_back(5);
    back->push_back(7);
    back->push_back(4);
    cube->addPrimitiveSet(back);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(color);
    colors->push_back(color);
    colors->push_back(color);
    colors->push_back(color);
    colors->push_back(color);
    colors->push_back(color);
    colors->push_back(color);
    colors->push_back(color);

    cube->setColorArray(colors);
    cube->setColorBinding(osg::Geometry::BIND_PER_VERTEX);


    osg::PolygonMode * polygonMode = new osg::PolygonMode;
    polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
    cube->getOrCreateStateSet()->setAttributeAndModes( polygonMode,
                                                                   osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );

    return cube;
}

osg::Geode* createAxis()
{
    osg::Geode*     geode    = new osg::Geode();
    osg::Geometry*  geometry = new osg::Geometry();
    osg::Vec3Array* vertices = new osg::Vec3Array();
    osg::Vec4Array* colors   = new osg::Vec4Array();

    vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    vertices->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
    vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    vertices->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
    vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    vertices->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));

    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));

    geometry->setVertexArray(vertices);
    geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));
    geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, false);

    geode->addDrawable(geometry);

    return geode;
}



// This idea here is to compute AABB by bone for a skelton
class ComputeAABBOnBoneVisitor : public osg::NodeVisitor {

public:
    ComputeAABBOnBoneVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _firstCall(true), _root(0) {}

#define  handleError() \
        if(_firstCall) { \
            std::cout << "ComputeAABBOnBoneVisitor must be call over a skeleton." << std::endl; \
            return; \
        }

    void apply(osg::Node &node) {
        handleError();
        osgAnimation::RigGeometry *rig = dynamic_cast<osgAnimation::RigGeometry*>(&node);
        if(rig) apply(*rig);
        traverse(node);
    }

    void apply(osg::Transform& node) {
        bool skl(false);
        if(_firstCall && (_root = dynamic_cast<osgAnimation::Skeleton*>(&node))) { _firstCall = false; skl=true; }

        osgAnimation::Bone * b = dynamic_cast<osgAnimation::Bone*>(&node);
        if(b) apply(*b);
        traverse(node);
        if(skl)
            computeBoundingBoxOnBones();
    }

    void apply(osgAnimation::Bone &bone) {
        handleError();
        _bones.push_back(&bone);
    }

    void apply(osgAnimation::RigGeometry &rig) {
        handleError();
        _rigGeometries.push_back(&rig);
    }

    inline void updateMesh() {
        //Update Bones
        osgUtil::UpdateVisitor up;
        _root->accept(up);

        //Update rigGeometries
        for (unsigned int i = 0, size = _rigGeometries.size(); i < size; i++) {
            osgAnimation::RigGeometry * rig = _rigGeometries.at(i);
            osg::Drawable::UpdateCallback * up = dynamic_cast<osg::Drawable::UpdateCallback*>(rig->getUpdateCallback());
            if(up) up->update(0, rig);
        }

        for (unsigned int i = 0, size = _rigGeometries.size(); i < size; i++) {
            osgAnimation::RigGeometry * rig = _rigGeometries.at(i);
            rig->update();
        }
    }

    void computeBoundingBoxOnBones() {

        updateMesh();

        //Compute AABB for each bone
        for ( std::vector<osgAnimation::Bone*>::iterator it = _bones.begin(); it != _bones.end(); it++) {
            osgAnimation::Bone *b = *it;
            osg::BoundingBox bb;
            //For each geometry
            for (std::vector<osgAnimation::RigGeometry*>::iterator it = _rigGeometries.begin(); it != _rigGeometries.end(); it++) {
                osgAnimation::RigGeometry *geom = *it;

                osg::Matrix mtxLocalToSkl =  geom->getWorldMatrices(_root).at(0);

                //For each Vertex influence
                osgAnimation::VertexInfluenceMap * infMap = geom->getInfluenceMap();
                osgAnimation::VertexInfluenceMap::iterator itMap = infMap->find(b->getName());
                if(itMap == infMap->end()) continue;

                osgAnimation::VertexInfluence vxtInf = (*itMap).second;
                osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());

                //Expand the boundingBox with each vertex
                for(int j = 0; j < vxtInf.size(); j++) {
                    if(vxtInf.at(j).second < 10e-2) continue; //Skip vertex if low weight
                    osg::Vec3 vx = vertices->at(vxtInf.at(j).first);
                    vx = mtxLocalToSkl* vx;
                    bb.expandBy(vx);
                }

                if(bb == osg::BoundingBox()) continue; //Compare initial and actual boundingBox

                osg::Matrix worldToBone = osg::Matrix::inverse(b->getWorldMatrices()[0]);

                osg::Geode *g = new osg::Geode;
                g->addDrawable(createBox(bb, worldToBone));
                b->addChild(g);
            }
        }

    }

    std::vector<osgAnimation::Bone*> _bones;
    std::vector<osgAnimation::RigGeometry*> _rigGeometries;
    bool _firstCall;
    osgAnimation::Skeleton *_root;
};


struct AnimationManagerFinder : public osg::NodeVisitor
{
    osg::ref_ptr<osgAnimation::BasicAnimationManager> _am;
    AnimationManagerFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node& node) {
        if (_am.valid())
            return;
        if (node.getUpdateCallback()) {
            osgAnimation::AnimationManagerBase* b = dynamic_cast<osgAnimation::AnimationManagerBase*>(node.getUpdateCallback());
            if (b) {
                _am = new osgAnimation::BasicAnimationManager(*b);
                return;
            }
        }
        traverse(node);
    }
};


struct AddHelperBone : public osg::NodeVisitor
{
    AddHelperBone() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Transform& node) {
        osgAnimation::Bone* bone = dynamic_cast<osgAnimation::Bone*>(&node);
        if (bone)
            bone->addChild(createAxis());
        traverse(node);
    }
};

struct FindNearestParentSkeleton : public osg::NodeVisitor
{
    osg::ref_ptr<osgAnimation::Skeleton> _root;
    FindNearestParentSkeleton() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_PARENTS) {}
    void apply(osg::Transform& node)
    {
        if (_root.valid())
            return;
        _root = dynamic_cast<osgAnimation::Skeleton*>(&node);
        traverse(node);
    }
};

struct FindSkeletons : public osg::NodeVisitor
{
    FindSkeletons() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Transform& node)
    {
        osgAnimation::Skeleton *skl = dynamic_cast<osgAnimation::Skeleton*>(&node);
        if(skl) _skls.push_back(skl);
        traverse(node);
    }

    std::vector<osgAnimation::Skeleton*> _skls;
};

struct FindRigGeometry : public osg::NodeVisitor
{

    std::vector<osgAnimation::RigGeometry*>  _geometry;

    FindRigGeometry() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node& node)
    {
        osgAnimation::RigGeometry *rig = dynamic_cast<osgAnimation::RigGeometry*>(&node);
        if(rig) {

            _geometry.push_back(rig);

            osg::PolygonMode * polygonMode = new osg::PolygonMode;
            polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
            rig->getOrCreateStateSet()->setAttributeAndModes( polygonMode,
                                                                           osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
        }
        traverse(node);
    }
};



class AnimationVisitor : public osgUtil::UpdateVisitor
{
public:
    AnimationVisitor() {}

    void apply(osg::Drawable& drawable) {
        apply(drawable.asGeometry());
    }

    void apply(osg::Geometry* geometry) {
        osgAnimation::RigGeometry* rig = dynamic_cast<osgAnimation::RigGeometry*>(geometry);
        if(rig) {
            apply(*rig);
        }
    }

    void apply(osgAnimation::RigGeometry& geometry) {
        // find skeleton
        osgAnimation::UpdateRigGeometry rigUpdater;
        rigUpdater.update(0, &geometry);
    }
};

// Here this idea is to compute a set of bounding box for a RigGeometry.
// This callback compute an AABB for a bone
//
// This bounding box is computed in skeleton space
//
//
//     [ Bone ]
//         |
//    [ Geometry (Box) ]
class ComputeBoundingBoxForBone : public osg::NodeCallback
{
public:
    ComputeBoundingBoxForBone() : _compute(0) {};

    void operator()(osg::Node* node, osg::NodeVisitor* nv) {

        if(  _compute > 2 ) return;

        osgAnimation::Bone * b = NULL;
        if( ! (b = dynamic_cast <osgAnimation::Bone *> ( node->getParent(0) )) ) return;


        //Find Skeleton
        FindNearestParentSkeleton fnps;
        node->accept(fnps);

        if(!fnps._root.valid()) return;
        //Find RigGeometries
        FindRigGeometry fg;
        osgAnimation::Skeleton * skl = fnps._root.get();
        skl->accept(fg);

        if(fg._geometry.empty()) return;

        osg::BoundingBox bb;
        //For Each Geometry
        for (int i = 0; i < fg._geometry.size(); i++) {
            osgAnimation::RigGeometry * geom = fg._geometry[i];

//            osgAnimation::UpdateRigGeometry rigUpdater;
//            rigUpdater.update(0, geom);



            osg::MatrixList ml = geom->getWorldMatrices(skl);
            osg::Matrix mtxLocalToSkl = ml.at(0);

            //For each Vertex influence
            osgAnimation::VertexInfluenceMap * infMap = geom->getInfluenceMap();
            osgAnimation::VertexInfluenceMap::iterator it = infMap->find(b->getName());
            if(it == infMap->end()) continue;
            //osgAnimation::VertexInfluence vi = it->second;

            osgAnimation::VertexInfluence vxtInf = (*it).second;
            if(vxtInf.size() == 0) continue; //Skip if no vertex influence for this bone

            osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());

            for(int j = 0; j < vxtInf.size(); j++) {

                if(vxtInf.at(j).second < 10e-2) continue; //Skip vertex if low weight
                osg::Vec3 vx = vertices->at(vxtInf.at(j).first);
                vx = mtxLocalToSkl* vx;
                bb.expandBy(vx);
            }
        }
        osg::Geode *g = static_cast<osg::Geode*> (node);

        osg::Matrix worldToBone = osg::Matrix::inverse(b->getWorldMatrices()[0]);   g->addDrawable(createBox(bb, worldToBone));

        //        osg::MatrixTransform * mt = dynamic_cast<osg::MatrixTransform*>(node->getParent(0));
        //        if( ! mt ) return;

        //        mt->setMatrix(worldToBone);
        //        osg::Vec3 min=bb._min, max=bb._max, center=bb.center();

               if(bb.center().x() == 0 && bb.center().y() == 0 && bb.center().z() == 0 ) return; //Adds no geometry if the initial BB not changed

        //        osg::ShapeDrawable * sd = new osg::ShapeDrawable();
        //        sd->setShape(new osg::Box(center, max.x() - min.x(), max.y() - min.y(),  max.z() - min.z()));
        //        g->addDrawable(sd);


        g->addDrawable(createBox(bb, worldToBone));


        _compute++;
    }

    int _compute;
};




// TO delete
//        Compute Bounding box here
//         And set it to the cubeShape
//        osg::Geode *g = static_cast<osg::Geode*> (node);
//        osg::ShapeDrawable * sd = static_cast<osg::ShapeDrawable*>(g->getDrawable(0));

//        //sd->setShape(new osg::Box( (min + max) / 2.f, max.x() - min.x(), max.y()-min.y(), max.z()-min.z()));
//        sd->setShape(new osg::Box(bb.center(), bb._max.x() - bb._min.x(), bb._max.y() - bb._min.y(), bb._max.z() - bb._min.z() ));

//        std::cout << b->getName() << std::endl;

//        std::cout << max.x() - min.x() << " " << max.y()-min.y() << " " << max.z()-min.z() << std::endl;

//        sd->setShape(new osg::Sphere((min + max) / 2.f, (max-min).length() / 2.f ));


struct SetBoxOnBoneVisitor : public osg::NodeVisitor
{
public:

    SetBoxOnBoneVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {};

    virtual void apply(osg::MatrixTransform& node)
    {
        osgAnimation::Bone * b = dynamic_cast<osgAnimation::Bone*>(&node);
        if(b) {

//            osg::MatrixTransform *mt = new osg::MatrixTransform();

////            osg::Box* unitCube = new osg::Box( osg::Vec3(0,0,0), 1.0f);
////            osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube);
//            osg::Geode* basicShapesGeode = new osg::Geode();
//            //basicShapesGeode->addDrawable(unitCubeDrawable);

//            //wireframe
//            osg::PolygonMode * polygonMode = new osg::PolygonMode;
//            polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
//            basicShapesGeode->getOrCreateStateSet()->setAttributeAndModes( polygonMode,
//                                                                           osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
//            mt->addChild(basicShapesGeode);
//            basicShapesGeode->setUpdateCallback(new ComputeBoundingBoxForBone);

//            b->addChild(mt);

//             osgAnimation::UpdateBone ub;
//             ub(b,0);

              osg::Geode * g = new osg::Geode;
              g->setUpdateCallback(new ComputeBoundingBoxForBone);
              b->addChild(g);
        }

        this->traverse(node);
    }
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is an example for viewing osgAnimation animations.");
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help","List command line options.");
    arguments.getApplicationUsage()->addCommandLineOption("--drawbone","draw helps to display bones.");

    if (arguments.read("-h") || arguments.read("--help"))
    {
        arguments.getApplicationUsage()->write(std::cout, osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 0;
    }

    if (arguments.argc()<=1)
    {
        arguments.getApplicationUsage()->write(std::cout, osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 1;
    }

    bool drawBone = false;
    if (arguments.read("--drawbone"))
        drawBone = true;

    osgViewer::Viewer viewer(arguments);
    osg::ref_ptr<osg::Group> group = new osg::Group();

    osg::Group* node = dynamic_cast<osg::Group*>(osgDB::readNodeFiles(arguments)); //dynamic_cast<osgAnimation::AnimationManager*>(osgDB::readNodeFile(psr[1]));
    if(!node)
    {
        std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }

    //Set Geometry Update callback boundingBox
//    SetBoxOnBoneVisitor bbv;
//    node->accept(bbv);

    FindSkeletons fs;
    node->accept(fs);

    if(fs._skls.size() > 0) {
        ComputeAABBOnBoneVisitor c3bv;
        fs._skls.at(0)->accept(c3bv);
    }


    // Set our Singleton's model.
    AnimationManagerFinder finder;
    node->accept(finder);
    if (finder._am.valid())
    {

        std::string playModeOpt;
        if (arguments.read("--play-mode", playModeOpt))
        {
            osgAnimation::Animation::PlayMode playMode = osgAnimation::Animation::LOOP;
            if (osgDB::equalCaseInsensitive(playModeOpt, "ONCE")) playMode = osgAnimation::Animation::ONCE;
            else if (osgDB::equalCaseInsensitive(playModeOpt, "STAY")) playMode = osgAnimation::Animation::STAY;
            else if (osgDB::equalCaseInsensitive(playModeOpt, "LOOP")) playMode = osgAnimation::Animation::LOOP;
            else if (osgDB::equalCaseInsensitive(playModeOpt, "PPONG")) playMode = osgAnimation::Animation::PPONG;

            for (osgAnimation::AnimationList::const_iterator animIter = finder._am->getAnimationList().begin();
                 animIter != finder._am->getAnimationList().end(); ++animIter)
            {
                (*animIter)->setPlayMode(playMode);
            }
        }

        node->setUpdateCallback(finder._am.get());
        AnimtkViewerModelController::setModel(finder._am.get());
    }
    else
    {
        osg::notify(osg::WARN) << "no osgAnimation::AnimationManagerBase found in the subgraph, no animations available" << std::endl;
    }

    if (drawBone) {
        osg::notify(osg::INFO) << "Add Bones Helper" << std::endl;
        AddHelperBone addHelper;
        node->accept(addHelper);
    }
    node->addChild(createAxis());

    AnimtkViewerGUI* gui    = new AnimtkViewerGUI(&viewer, WIDTH, HEIGHT, 0x1234);
    osg::Camera*     camera = gui->createParentOrthoCamera();

    node->setNodeMask(0x0001);

    group->addChild(node);
    group->addChild(camera);

    viewer.addEventHandler(new AnimtkKeyEventHandler());
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgWidget::MouseHandler(gui));
    viewer.addEventHandler(new osgWidget::KeyboardHandler(gui));
    viewer.addEventHandler(new osgWidget::ResizeHandler(gui, camera));
    viewer.setSceneData(group.get());

    viewer.setUpViewInWindow(40, 40, WIDTH, HEIGHT);

    return viewer.run();
}
