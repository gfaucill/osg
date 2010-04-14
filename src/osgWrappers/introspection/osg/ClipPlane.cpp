// ***************************************************************************
//
//   Generated automatically by genwrapper.
//   Please DO NOT EDIT this file!
//
// ***************************************************************************

#include <osgIntrospection/ReflectionMacros>
#include <osgIntrospection/TypedMethodInfo>
#include <osgIntrospection/StaticMethodInfo>
#include <osgIntrospection/Attributes>

#include <osg/ClipPlane>
#include <osg/CopyOp>
#include <osg/Object>
#include <osg/Plane>
#include <osg/State>
#include <osg/StateAttribute>
#include <osg/Vec4d>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(osg::ClipPlane)
	I_DeclaringFile("osg/ClipPlane");
	I_BaseType(osg::StateAttribute);
	I_Constructor0(____ClipPlane,
	               "",
	               "");
	I_Constructor1(IN, unsigned int, no,
	               Properties::NON_EXPLICIT,
	               ____ClipPlane__unsigned_int,
	               "",
	               "");
	I_Constructor2(IN, unsigned int, no, IN, const osg::Vec4d &, plane,
	               ____ClipPlane__unsigned_int__C5_Vec4d_R1,
	               "",
	               "");
	I_Constructor2(IN, unsigned int, no, IN, const osg::Plane &, plane,
	               ____ClipPlane__unsigned_int__C5_Plane_R1,
	               "",
	               "");
	I_Constructor5(IN, unsigned int, no, IN, double, a, IN, double, b, IN, double, c, IN, double, d,
	               ____ClipPlane__unsigned_int__double__double__double__double,
	               "",
	               "");
	I_ConstructorWithDefaults2(IN, const osg::ClipPlane &, cp, , IN, const osg::CopyOp &, copyop, osg::CopyOp::SHALLOW_COPY,
	                           ____ClipPlane__C5_ClipPlane_R1__C5_CopyOp_R1,
	                           "Copy constructor using CopyOp to manage deep vs shallow copy. ",
	                           "");
	I_Method0(osg::Object *, cloneType,
	          Properties::VIRTUAL,
	          __osg_Object_P1__cloneType,
	          "Clone the type of an attribute, with Object* return type. ",
	          "Must be defined by derived classes. ");
	I_Method1(osg::Object *, clone, IN, const osg::CopyOp &, x,
	          Properties::VIRTUAL,
	          __osg_Object_P1__clone__C5_osg_CopyOp_R1,
	          "Clone an attribute, with Object* return type. ",
	          "Must be defined by derived classes. ");
	I_Method1(bool, isSameKindAs, IN, const osg::Object *, obj,
	          Properties::VIRTUAL,
	          __bool__isSameKindAs__C5_osg_Object_P1,
	          "Return true if this and obj are of the same kind of object. ",
	          "");
	I_Method0(const char *, libraryName,
	          Properties::VIRTUAL,
	          __C5_char_P1__libraryName,
	          "Return the name of the attribute's library. ",
	          "");
	I_Method0(const char *, className,
	          Properties::VIRTUAL,
	          __C5_char_P1__className,
	          "Return the name of the attribute's class type. ",
	          "");
	I_Method0(osg::StateAttribute::Type, getType,
	          Properties::VIRTUAL,
	          __Type__getType,
	          "Return the Type identifier of the attribute's class type. ",
	          "");
	I_Method1(int, compare, IN, const osg::StateAttribute &, sa,
	          Properties::VIRTUAL,
	          __int__compare__C5_StateAttribute_R1,
	          "Return -1 if *this < *rhs, 0 if *this==*rhs, 1 if *this>*rhs. ",
	          "");
	I_Method0(unsigned int, getMember,
	          Properties::VIRTUAL,
	          __unsigned_int__getMember,
	          "Return the member identifier within the attribute's class type. ",
	          "Used for light number/clip plane number etc. ");
	I_Method1(bool, getModeUsage, IN, osg::StateAttribute::ModeUsage &, x,
	          Properties::VIRTUAL,
	          __bool__getModeUsage__StateAttribute_ModeUsage_R1,
	          "Return the modes associated with this StateAttribute. ",
	          "");
	I_Method1(void, setClipPlane, IN, const osg::Plane &, plane,
	          Properties::NON_VIRTUAL,
	          __void__setClipPlane__C5_Plane_R1,
	          "Set the clip plane with the given Plane. ",
	          "");
	I_Method4(void, setClipPlane, IN, double, a, IN, double, b, IN, double, c, IN, double, d,
	          Properties::NON_VIRTUAL,
	          __void__setClipPlane__double__double__double__double,
	          "Defines the plane as [ a b c d ]. ",
	          "");
	I_Method1(void, setClipPlane, IN, const osg::Vec4d &, plane,
	          Properties::NON_VIRTUAL,
	          __void__setClipPlane__C5_Vec4d_R1,
	          "Set the clip plane with the given Vec4. ",
	          "");
	I_Method0(const osg::Vec4d &, getClipPlane,
	          Properties::NON_VIRTUAL,
	          __C5_Vec4d_R1__getClipPlane,
	          "Gets the clip plane as a Vec4d. ",
	          "");
	I_Method1(void, setClipPlaneNum, IN, unsigned int, num,
	          Properties::NON_VIRTUAL,
	          __void__setClipPlaneNum__unsigned_int,
	          "Sets the clip plane number. ",
	          "");
	I_Method0(unsigned int, getClipPlaneNum,
	          Properties::NON_VIRTUAL,
	          __unsigned_int__getClipPlaneNum,
	          "Gets the clip plane number. ",
	          "");
	I_Method1(void, apply, IN, osg::State &, state,
	          Properties::VIRTUAL,
	          __void__apply__State_R1,
	          "Applies the clip plane's state to the OpenGL state machine. ",
	          "");
	I_SimpleProperty(const osg::Vec4d &, ClipPlane, 
	                 __C5_Vec4d_R1__getClipPlane, 
	                 __void__setClipPlane__C5_Vec4d_R1);
	I_SimpleProperty(unsigned int, ClipPlaneNum, 
	                 __unsigned_int__getClipPlaneNum, 
	                 __void__setClipPlaneNum__unsigned_int);
	I_SimpleProperty(unsigned int, Member, 
	                 __unsigned_int__getMember, 
	                 0);
	I_SimpleProperty(osg::StateAttribute::Type, Type, 
	                 __Type__getType, 
	                 0);
END_REFLECTOR
