/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"


#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
#include<iostream>
using namespace std;

#include "dominos.hpp"

namespace cs296
{  /** The constructor for the dominos_t class.
 *  This constructor includes declarations and definition oll the
 *	dynamic as well as static elements present in the final simulation
 */
  dominos_t::dominos_t()
  {
    //Ground
	/** <b>Block 1 : GROUND</b>*/
    b2Body* b1;
    {
      
		/// \b shape is a \b b2EdgeShape object. Its length is set to 180 units, 
		///90units on both sides of origin along the x-axis
		b2EdgeShape shape;
		shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
		/// A \b b2BodyDef type object \b bd is declared and created in the world.
		b2BodyDef bd;
		b1 = m_world->CreateBody(&bd);
		/// \b bd is given the characterstics as defined by \b shape, density as zero and added to the simulation world.
		b1->CreateFixture(&shape, 0.0f);
    }
    
    
    //The pulley system
    /** <b>Block 2 : PULLEY SYSTEM</b>*/
    {
   
		b2BodyDef *bd = new b2BodyDef;
		bd->position.Set(-3.0,16);
		bd->type = b2_dynamicBody;
		bd->fixedRotation = true;
		b2FixtureDef *fd1 = new b2FixtureDef;
		///To define the characterstics of the sphere ball a \b b2FixtureDef type variable \b ballfd is defined.
		///shape = \b circle defined above , density = 50 units , friction = 0.5 , restitution = 0.0
		fd1->density = 50;
		fd1->friction = 0.5;
		fd1->restitution = 0.f;
		fd1->shape = new b2CircleShape;
		///Its shape is set to a circle of radius 1 unit as described by the \b b2CircleShap object circle.
		b2CircleShape circ1;
		/// position set to (-32.0, 0.2) in default X-Y plane.
		circ1.m_p.Set(-32.f, 0.2f);
		circ1.m_radius = 2.f;
		fd1->shape = &circ1;
		b2Body* box1 = m_world->CreateBody(bd);
		// box1->CreateFixture(fd1);
		//The bar
		
		/**<b>SUB_BLOCK 1 : THE BAR</b>*/
			///The \b b2Bodydef type variable \b bd is reused and set to the position (10 , 15) in default X-Y plane cordinate system.
     	fd1 = new b2FixtureDef;
		fd1->density = 50;
		fd1->friction = 0.5;
		fd1->restitution = 0.f;
		fd1->shape = new b2CircleShape;
		circ1.m_p.Set(-28.f, 0.2f);
		circ1.m_radius = 1.f;
		fd1->shape = &circ1;
		b2Body* box2 = m_world->CreateBody(bd);
        box2->CreateFixture(fd1);
        // The pulley joint
        
        /**<b>SUB_BLOCK 2 : THE PULLEY JOINT</b>*/
		b2PulleyJointDef* myjoint = new b2PulleyJointDef();
		//b2Vec2 worldAnchorOnBody1(-35, 17); // Anchor point on body 1 in world axis
		//b2Vec2 worldAnchorOnBody2(-31, 17); // Anchor point on body 2 in world axis
		b2Vec2 worldAnchorGround1(-35, 22); 
			/// Anchor point for ground 1 in world axis
		b2Vec2 worldAnchorGround2(-31, 22); 
			/// Anchor point for ground 2 in world axis
		float32 ratio = 1.0f; 
			/// Define ratio of string on the two sides of the pulley as 1.
			///Initialise \b myjoint by passing parameters in the order of bodyA , bodyB , groundAnchor1 , groundAnchor2 , bodyA-anchor , bodyB-anchor and ratio of strings.
		myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, b2Vec2(-35,10), box2->GetWorldCenter(), ratio);
			///finally create a joint in the simulation world.
		m_world->CreateJoint(myjoint);
      
        // Human
		/**<b>SUB_BLOCK 3 : HUMAN</b>*/
			/**<b>SUB_SUB_BLOCK 1 : SHOULDER</b>*/
			
		b2PolygonShape shape6;
		shape6.SetAsBox(0.3f, 1.f,b2Vec2(0,0),-0.5);
		b2FixtureDef *fd6 = new b2FixtureDef;
		fd6->shape = &shape6;
		fd6->density = 50.0f;
				///The shoulder is referenced in the world as an object of \b b2Body named \b should.
				///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd6.
				///density = 50 unit, shape = box shape defined by the \b shape6 variable above.
		b2BodyDef should;
		should.position.Set(37.5f, 8.f);
		b2Body* shoulder = m_world->CreateBody(&should);
		shoulder->CreateFixture(fd6);
      
			/**<b>SUB_SUB_BLOCK 2 : ELBOW</b>*/
		b2PolygonShape shape7;
		shape7.SetAsBox(0.3f, 1.5f,b2Vec2(-0.3,-1.2),-0.4);
		b2FixtureDef *fd7 = new b2FixtureDef;
		fd7->shape = &shape7;
		fd7->density = 50.0f;
				///The shoulder is referenced in the world as an object of \b b2Body named \b elb.
				///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd7.
				///density = 50 unit, shape = box shape defined by the \b shape7 variable above.
		b2BodyDef elb;
		elb.type = b2_dynamicBody;
		elb.position.Set(37.f, 7.f);
		b2Body* elbow = m_world->CreateBody(&elb);
		elbow->SetGravityScale(-8);
		//elbow->SetAngularVelocity(-1.0);
		elbow->CreateFixture(fd7);
   
        ///Definition of a \b b2RevoluteJointDef object \b elbowJoint. The joint joints \b shoulder to \b elbow.
		b2RevoluteJointDef elbowJoint;
		elbowJoint.bodyA = shoulder;
		elbowJoint.referenceAngle = 2;
		elbowJoint.bodyB = elbow;
		///The local anchor of  the body \b shoulder  is set to (-0.5,-0.7).
		elbowJoint.localAnchorA.Set(-0.5,-0.7);
		///The local anchor of  the body \b elbow  is set to (0,0).
		elbowJoint.localAnchorB.Set(0,0);
		elbowJoint.collideConnected = false;
      
		//elbowJoint.enableMotor = true;
		//elbowJoint.maxMotorTorque = 20;
		//elbowJoint.motorSpeed = 50;
		m_world->CreateJoint(&elbowJoint);
		b2PulleyJointDef* joint = new b2PulleyJointDef();
     	b2Vec2 worldAnchorOnBody11(36.5, 5); 
     	/// Anchor point on body 1 in world axis
		//b2Vec2 worldAnchorOnBody21(-35, 17); // Anchor point on body 2 in world axis
		b2Vec2 worldAnchorGround11(37.5, 2); 
		/// Anchor point for ground 1 in world axis
		b2Vec2 worldAnchorGround21(-35, 2); 
		/// Anchor point for ground 2 in world axis
		//float32 ratio = 1.0f; // Define ratio
        ///Initialise \b myjoint by passing parameters in the order of bodyA , bodyB , groundAnchor1 , groundAnchor2 , bodyA-anchor , bodyB-anchor and ratio of strings.
		joint->Initialize(elbow, box1, worldAnchorGround11, worldAnchorGround21, worldAnchorOnBody11, b2Vec2(-35,10), 1.0f);
		///finally create a joint in the simulation world.
		m_world->CreateJoint(joint);
      
  
      
    }
    
     // platform 
    /**<b>BLOCK 3 : PLATFORM</b>*/
    
    {
    	
      
		b2BodyDef bd;
		///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (-26.0, 18.0) units in X-Y plane.
		bd.position.Set(-26.0f, 18.0f);
		b2Body* body = m_world->CreateBody(&bd);
		/// \b shape is an object of \b b2PolygonShape type. \b shape is given a box shape to represent a platform.
		///Width = 4.0 * 2 = 8.0 units and height = 0.2 * 2 = 0.4 units.
		b2PolygonShape shape;
		shape.SetAsBox(4.f, 0.2f);
		///The platform is referenced in the world as an object of \b b2Body named \b body.
		///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd.
		///density = 50 unit, shape = box shape defined by the \b shape variable above.
		b2FixtureDef *fd = new b2FixtureDef;
		fd->density = 50.0f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
		body->CreateFixture(fd);
		///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (-11.5, 14.0) units in X-Y plane.
		bd.position.Set(-11.5f, 14.0f);
		body = m_world->CreateBody(&bd);
		body->CreateFixture(fd);
		///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (3.0, 10.0) units in X-Y plane.
		bd.position.Set(3.f, 10.0f);
		body = m_world->CreateBody(&bd);
		body->CreateFixture(fd);
		
    }
    
     // platform 
    /**<b>BLOCK 4 : PLATFORM</b>*/
    {
    	b2BodyDef bd;
    	///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (-18.5, 15.8) units in X-Y plane.
		bd.position.Set(-18.5f, 15.8f);
		b2Body* body = m_world->CreateBody(&bd);
		b2PolygonShape shape;
		b2Vec2 center;
		center.Set(0.f,0.f);
		/// \b shape is an object of \b b2PolygonShape type. \b shape is given a box shape to represent a platform.
		///Width = 4.0 * 2 = 8.0 units and height = 0.2 * 2 = 0.4 units.
		shape.SetAsBox(4.f, 0.2f,center , -10);
		///The platform is referenced in the world as an object of \b b2Body named \b body.
		///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd.
		///density = 50 unit, shape = box shape defined by the \b shape variable above.
		b2FixtureDef *fd = new b2FixtureDef;
		fd->density = 50.0f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
		body->CreateFixture(fd);
		///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (-4.2, 11.8) units in X-Y plane.
		bd.position.Set(-4.2f, 11.8f);
		body = m_world->CreateBody(&bd);
		body->CreateFixture(fd);
    
    }
    
    // platform with balls on it
    /**<b>BLOCK 5 : PLATFORM WITH BALLS</b>*/
	{
    
		///The sphere body on the platform is a \b b2Body object named by \b sbody.
		///Its shape is set to a circle of radius 1 unit as described by the \b b2CircleShap object circle.
    	b2Body* sbody;
		b2CircleShape circle;
		circle.m_radius = 1.0;
		///To define the characterstics of the sphere ball a \b b2FixtureDef type variable \b ballfd is defined.
		///shape = \b circle defined above , density = 20 units , friction = 0.2 , restitution = 0.0
		b2FixtureDef ballfd;
		ballfd.shape = &circle;
		ballfd.density = 20.0f;
		ballfd.friction = 0.2f;
		ballfd.restitution = 0.0f;
		///To define the body of spherical ball an object of \b b2Bodydef named \b ballbd is defined. Its type is set to \b b2_dynamicBody as it is mobile.
		b2BodyDef ballbd;
		ballbd.type = b2_dynamicBody;
		///The spherical ball is set to the position (-29.5,19.0) in default X-Y plane.
		ballbd.position.Set(-29.5f, 19.0f);
		sbody = m_world->CreateBody(&ballbd);
		sbody->CreateFixture(&ballfd);
		///The spherical ball is set to the position (5.5,11.0) in default X-Y plane.
		ballbd.position.Set(5.5f, 11.0f);
		sbody = m_world->CreateBody(&ballbd);
		sbody->CreateFixture(&ballfd);
		///The spherical ball is set to the position (-8.0,15.0) in default X-Y plane.
		ballbd.position.Set(-8.0f, 15.0f);
		sbody = m_world->CreateBody(&ballbd);
		sbody->CreateFixture(&ballfd);
      
    }
    
    // projectile platform with a ball on it
    /**<b>BLOCK 6 : PROJECTILE PLATFORM WITH A BALL</b>*/
    {
	   
		/**<b>SUB_BLOCK 1 : PLATFORM</b>*/
			///To define the body of platform a \b b2PolygonShape \b shape is defined. Its position is set to (8.2, 0.2) units in X-Y plane.
		b2PolygonShape shape; 
		shape.SetAsBox(8.2f, 0.2f);
			///To define the body of platform a \b b2PolygonShape \b shape is defined. Its position is set to (8.,1.) units in X-Y plane relative to (8.2, 0.2) units in X-Y plane .
		b2PolygonShape shape1;
		shape1.SetAsBox(0.2f, 1.0f , b2Vec2(8.f,1.f),0);
			///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (18.0, 4.0) units in X-Y plane.
		b2BodyDef bd;
		bd.position.Set(18.0f, 4.0f);
			///The platform is referenced in the world as an object of \b b2Body named \b body.
			///Its type is set to \b b2_dynamicBody as it is mobile.
			///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd.
			///density = 1 unit, shape = box shape defined by the \b shape variable above.
		bd.type = b2_dynamicBody;
		b2Body* body = m_world->CreateBody(&bd);
		b2FixtureDef *fd = new b2FixtureDef;
		fd->density = 1.f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
			///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd1.
			///density = 1 unit, shape = box shape defined by the \b shape1 variable above.
		b2FixtureDef *fd1 = new b2FixtureDef;
		fd1->density = 1.f;
		fd1->shape = new b2PolygonShape;
		fd1->shape = &shape1;
      
		body->CreateFixture(fd);
		body->CreateFixture(fd1);      

		b2PolygonShape shape2;
		shape2.SetAsBox(0.2f, 4.0f);
		b2BodyDef bd2;
		bd2.position.Set(16.0f, 4.0f);
		b2Body* body2 = m_world->CreateBody(&bd2);
			///Definition of a \b b2RevoluteJointDef object \b jointDef. The joint joints \b body to \b body2.
		b2RevoluteJointDef jointDef;
		jointDef.bodyA = body;
		jointDef.bodyB = body2;
			///The local anchors of both the bodies \b body and \b body2 are set to (-2,0).
		jointDef.localAnchorA.Set(-2,0);
		jointDef.localAnchorB.Set(-2,0);
		jointDef.collideConnected = false;
			///The \b jointDef member collide connected is set to false to prevent collision of connected bodies.
		m_world->CreateJoint(&jointDef);
      
		/**<b>SUB_BLOCK 2 : BALL</b>*/
			///The sphere body on the platform is a \b b2Body object named by \b sbody.
      
		b2Body* sbody;
		b2CircleShape circle;
			///Its shape is set to a circle of radius 1 unit as described by the \b b2CircleShap object circle.
		circle.m_radius = 1.0;
			///To define the characterstics of the sphere ball a \b b2FixtureDef type variable \b ballfd is defined.
			///shape = \b circle defined above , density = 1.5 units
		b2FixtureDef ballfd;
		ballfd.shape = &circle;
		ballfd.density = 1.5f;
		//ballfd.friction = 0.0f;
		//ballfd.restitution = 0.0f;
			///To define the body of spherical ball an object of \b b2Bodydef named \b ballbd is defined. Its type is set to \b b2_dynamicBody as it is mobile.
		b2BodyDef ballbd;
		ballbd.type = b2_dynamicBody;
			///The spherical ball is set to the position (20.0, 5.4) in default X-Y plane.
		ballbd.position.Set(20.0f, 5.4f);
		sbody = m_world->CreateBody(&ballbd);
		sbody->CreateFixture(&ballfd);
		
		/**<b>SUB_BLOCK 3 : SUPPORT</b>*/ 
			///To define the body of platform a \b b2PolygonShape \b shape3 is defined.
		b2PolygonShape shape3;
		shape3.SetAsBox(0.2f, 0.2f);
		b2BodyDef bd3;
			/// Its position is set to (21.0, 3.58) units in X-Y plane.
		bd3.position.Set(21.0f, 3.58f);
		b2Body* body3 = m_world->CreateBody(&bd3);
		b2FixtureDef *fd3 = new b2FixtureDef;
		fd3->shape = &shape3;
		body3->CreateFixture(fd3);
      
    }
    
    //  platform on the second level
    /**<b>BLOCK 7 :  PLATFORM ON THE SECOND LEVEL</b>*/
    {
    	
    	///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (0.0, 24.0) units in X-Y plane.
    	b2BodyDef bd;
      
		bd.position.Set(0.0f, 24.0f);
		b2Body* body = m_world->CreateBody(&bd);
      
		b2PolygonShape shape;
		///Width = 4.0 * 2 = 8.0 units and height = 0.2 * 2 = 0.4 units.
		shape.SetAsBox(4.f, 0.2f);
		///The platform is referenced in the world as an object of \b b2Body named \b body.
		///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd.
		///density = 50 unit, shape = box shape defined by the \b shape variable above.
		b2FixtureDef *fd = new b2FixtureDef;
		fd->density = 50.0f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
		body->CreateFixture(fd);
      
		shape.SetAsBox(4.f, 0.2f ,b2Vec2(-7.6,-2), -5.8);
		body->CreateFixture(fd);
      
    }
    
    //  Joint on the second level
    /**<b>BLOCK 8 :  JOINT ON THE SECOND LEVEL</b>*/
	{
    	
		/**<b>SUB_BLOCK 1 : SUPPORT TO T-SHAPE PLATFORM</b>*/
			///To define the body of platform a \b b2PolygonShape \b shape is defined. 
		b2PolygonShape shape2;
		shape2.SetAsBox(2.f, 0.2f);
			///To define the body of platform a \b b2BodyDef object \b bd2 is defined. 
			///Its position is set to (-12.6, 20.2) units in X-Y plane.
		b2BodyDef bd2;
		bd2.position.Set(-12.6f, 20.2f);
		b2Body* body2 = m_world->CreateBody(&bd2);
			///The platform is referenced in the world as an object of \b b2Body named \b body2.
			///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd.
			///density = 20 unit, shape = box shape defined by the \b shape variable above.
		b2FixtureDef *fd2 = new b2FixtureDef;
		fd2->density = 20.0f;
		fd2->shape = new b2PolygonShape;
		fd2->shape = &shape2;
		body2->CreateFixture(fd2);
       
		/**<b>SUB_BLOCK 2 : T-SHAPE PLATFORM</b>*/
			///To define the body of platform a \b b2PolygonShape \b shape1 is defined. 
    	b2PolygonShape shape1;
		shape1.SetAsBox(4.f, 0.2f,b2Vec2(0,6), 0 );
			///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd1.
			///density = 1 unit, shape = box shape1 defined by the \b shape variable above.
		b2FixtureDef *fd1 = new b2FixtureDef;
		fd1->density = 1.f;
		fd1->shape = new b2PolygonShape;
		fd1->shape = &shape1;
    		///To define the body of platform a \b b2BodyDef object \b bd is defined. Its position is set to (-13.0f, 20.5f) units in X-Y plane.
			/// its type is dynamic body to make it mobile
    	b2BodyDef bd;
    	bd.type = b2_dynamicBody;
		bd.position.Set(-13.0f, 20.5f);
		b2Body* body = m_world->CreateBody(&bd);
		b2PolygonShape shape;
		shape.SetAsBox(0.2f, 6.0f);
		b2FixtureDef *fd = new b2FixtureDef;
		fd->density = 1.f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
      
		body->CreateFixture(fd);
		body->CreateFixture(fd1);
     
			///Definition of a \b b2RevoluteJointDef object \b jointDef. The joint joints \b body to \b body2.
		b2RevoluteJointDef jointDef;
		jointDef.bodyA = body;
		jointDef.bodyB = body2;
			///The local anchor of  the body \b body  is set to (0,-4).
		jointDef.localAnchorA.Set(0,-4);
			///The local anchor of  the body \b body2  is set to (-0.4,0).
		jointDef.localAnchorB.Set(-0.4,0);
			///The \b jointDef member collide connected is set to false to prevent collision of connected bodies.
		m_world->CreateJoint(&jointDef);
		jointDef.collideConnected = false;
		m_world->CreateJoint(&jointDef);
		
    }
    
	//balls on the moving hinged edge
    /**<b>BLOCK 9 :  BALL ON THE MOVING HINGED EDGE</b>*/
    {
    	
		///The sphere body on the platform is a \b b2Body object named by \b sbody.
		///Its shape is set to a circle of radius 1 unit as described by the \b b2CircleShap object circle.
    	b2Body* sbody;
		b2CircleShape circle;
		circle.m_radius = 1.0;
		///To define the characterstics of the sphere ball a \b b2FixtureDef type variable \b ballfd is defined.
		///shape = \b circle defined above , density = 10 units , friction = 0.2 , restitution = 0.0
		b2FixtureDef ballfd;
		ballfd.shape = &circle;
		ballfd.density = 10.0f;
		ballfd.friction = 0.2f;
		ballfd.restitution = 0.0f;
		///To define the body of spherical ball an object of \b b2Bodydef named \b ballbd is defined.
		/// Its type is set to \b b2_dynamicBody as it is mobile.
		b2BodyDef ballbd;
		ballbd.type = b2_dynamicBody;
		///The spherical ball is set to the position (-14.5, 31.0) in default X-Y plane.
		ballbd.position.Set(-14.5f, 31.f);
		sbody = m_world->CreateBody(&ballbd);
		sbody->CreateFixture(&ballfd);
		///The spherical ball is set to the position (-11.5, 31.0) in default X-Y plane.
		ballbd.position.Set(-11.5f, 31.f);
		sbody = m_world->CreateBody(&ballbd);
		sbody->CreateFixture(&ballfd);
    	
    }
    
    //dominos platform
    /**<b>BLOCK 10 :  DOMINOS PLATFORM </b>*/
    {
    	
		///To define the body of platform a \b b2BodyDef object \b bd is defined.
		/// Its position is set to (-15.0, 27.0) units in X-Y plane.
		b2BodyDef bd;
		bd.position.Set(-15.0f, 27.0f);
		b2Body* body = m_world->CreateBody(&bd);
        /// \b shape is an object of \b b2PolygonShape type. \b shape is given a box shape to represent a platform.
		///Width = 7.0 * 2 = 14.0 units and height = 0.2 * 2 = 0.4 units.
		b2PolygonShape shape;
		shape.SetAsBox(7.0f, 0.2f);
		///The platform is referenced in the world as an object of \b b2Body named \b body2.
		///The corresponding fixture properties are defined by the \b b2FixtureDef variable \b fd.
		///density = 50 unit, shape = box shape defined by the \b shape variable above.
		b2FixtureDef *fd = new b2FixtureDef;
		fd->density = 50.0f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
      
     
		shape.SetAsBox(3.f, 0.2f ,b2Vec2(-8.6,-1.5), -5.8);
		body->CreateFixture(fd);
      
		shape.SetAsBox(8.f, 0.2f ,b2Vec2(-18.6,-2.8), 0);
		body->CreateFixture(fd);
    
    }
    
    //dominoes and ball
	/**<b>BLOCK 10 : DOMINOES AND BALL </b> */
    {
		
		b2PolygonShape shape;
		shape.SetAsBox(0.2f, 1.5f);
	
		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 20.0f;
		fd.friction = 0.1f;
		
		for (int i = 0; i < 10; ++i)
		{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(-37.0f + 1.0f * i, 25.25f);
		b2Body* body = m_world->CreateBody(&bd);
		body->CreateFixture(&fd);
		}
	
		b2Body* sbody;
		b2CircleShape circle;
		circle.m_radius = 1.0;
	
		b2FixtureDef ballfd;
		ballfd.shape = &circle;
      ballfd.density = 15.0f;
      ballfd.friction = 0.2f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-40.5f, 24.25f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    
    }

    {
		
		
    	b2BodyDef bd;
      bd.position.Set(-56.0f, 16.0f);
      b2Body* body = m_world->CreateBody(&bd);
      
      b2PolygonShape shape;
      shape.SetAsBox(4.f, 0.2f);
      
      b2PolygonShape shape1;
      shape1.SetAsBox(0.3f, 6.0f,b2Vec2(-3.8,6.0), 0);
      
      b2PolygonShape shape2;
      shape2.SetAsBox(0.3f, 2.0f,b2Vec2(4,1.5), 0);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 50.0f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &shape1;
      
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 50.0f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape2;
      
      body->CreateFixture(fd);
      body->CreateFixture(fd1);      
      body->CreateFixture(fd2);
    	
    	
    }
    
    {
    	b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 15.0f;
      ballfd.friction = 0.2f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-55.5f, 19.25f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      sbody->SetGravityScale(-1);
      
      ballbd.position.Set(-56.9f, 19.25f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      sbody->SetGravityScale(-1);
      
      
    }
    
    {
    	b2BodyDef bd;
      bd.position.Set(-51.4f, 20.3f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      
      b2PolygonShape shape;
      shape.SetAsBox(8.f, 0.2f);
      
      b2PolygonShape shape1;
      shape1.SetAsBox(3.f, 1.0f,b2Vec2(-1.3,1), 0);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 50.0f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &shape1;
      
      
      body->CreateFixture(fd);
      body->CreateFixture(fd1);      
    }
   
    {
	   //third level
    	
    	b2BodyDef bd;
      bd.position.Set(-56.5f, 57.0f);
      b2Body* body = m_world->CreateBody(&bd);
      
      b2PolygonShape shape;
      shape.SetAsBox(1.f, 0.2f);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      
      shape.SetAsBox(5.f, 0.2f ,b2Vec2(6.0,-0.65), 3.0);
      body->CreateFixture(fd);
		
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 75.0f;
      ballfd.friction = 0.1f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-57.0f, 58.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
    
    {
		
		
    	b2BodyDef bd;
      bd.position.Set(-33.5f, 50.5f);
      bd.angle=3.0f;
      b2Body* body = m_world->CreateBody(&bd);
      
      b2PolygonShape shape,shape1;
      shape.SetAsBox(5.f, 0.2f);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      
     shape.SetAsBox(2.5f, 0.2f ,b2Vec2(2.7,-2.25), 0.0);
     body->CreateFixture(fd);
      
      b2BodyDef bd1;
      bd1.position.Set(-33.5f, 50.5f);
      bd1.type = b2_dynamicBody;
      b2Body* body1 = m_world->CreateBody(&bd1);
     
       b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 25.0f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &shape1;
     shape.SetAsBox(7.0f, 0.2f ,b2Vec2(-4.7,3.25), 3.0f);
     shape1.SetAsBox(1.25f, 0.2f ,b2Vec2(2.1,1.1), 4.50f);
     body1->CreateFixture(fd);
     body1->CreateFixture(fd1);
     shape.SetAsBox(2.5f, 0.3f ,b2Vec2(-5,2.6), 1.57f);
      body->CreateFixture(fd);
  }
   {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.6;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 20.0f;
      ballfd.friction = 0.1f;
      ballfd.restitution = 0.0f;
	
      for (int i = 0; i < 4; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-35.5f + i*1.2, 51.5f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }
    {
	  b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.1f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_kinematicBody;
      ballbd.position.Set(-28.4f, 43.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->SetTransform( b2Vec2( -28.4f, 43.0f),0.2 );
      sbody->SetAngularVelocity( -1.5);
      sbody->CreateFixture(&ballfd);
      
      b2BodyDef bd;
      bd.position.Set(-27.6f, 50.0f);
      bd.angle=1.44f;
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      
      
      b2PolygonShape shape,shape1;
      shape.SetAsBox(4.0f, 0.7f);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->SetGravityScale(-1);
      
      body->CreateFixture(fd);
   

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = sbody;
      jointDef.localAnchorA.Set(-3.8,0);
      jointDef.localAnchorB.Set(-1.0,1.0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
	
	  b2BodyDef bd2;
      bd2.position.Set(-26.7f, 47.7f);
      bd2.angle=1.44f;
      b2Body* body3 = m_world->CreateBody(&bd2);
      
      
      b2PolygonShape shape3;
      shape3.SetAsBox(3.4f, 0.2f);
      
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.0f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape3;
      body3->CreateFixture(fd2);
      
     }
	  {
      b2BodyDef bd2;
      bd2.position.Set(-21.3f, 50.8f);
	   b2Body* body = m_world->CreateBody(&bd2); 
	  b2PolygonShape shape;
       
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
   	  shape.SetAsBox(5.0f, 0.2f ,b2Vec2(0,0), 3.0f);
   	  body->CreateFixture(fd);
   	  shape.SetAsBox(3.0f, 0.2f ,b2Vec2(4.7,-4.0), 1.57f);
   	  body->CreateFixture(fd);
   	  shape.SetAsBox(3.0f, 0.2f ,b2Vec2(1.6,-7.3), -3.0f);
   	  body->CreateFixture(fd);
   	  shape.SetAsBox(4.25f, 0.2f ,b2Vec2(6.9,-2.4), 1.57f);
   	  body->CreateFixture(fd);
   	  shape.SetAsBox(4.0f, 0.2f ,b2Vec2(3.0,-9.3), -3.0f);
   	  body->CreateFixture(fd);
	  shape.SetAsBox(5.0f, 0.2f ,b2Vec2(11.5,-9.5), 3.0f);
   	  body->CreateFixture(fd);
      shape.SetAsBox(3.5f, 0.2f ,b2Vec2(19.45,-10.4), -0.05);
   	  body->CreateFixture(fd);
      }
      {
	  b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.1f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_kinematicBody;
      ballbd.position.Set(-24.0f, 42.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->SetTransform( b2Vec2( -24.0f, 42.0f),0.2 );
      sbody->SetAngularVelocity( -1.50);
      sbody->CreateFixture(&ballfd);
      
      b2BodyDef bd;
      bd.position.Set(-18.9f, 42.0f);
      bd.angle=-3.0f;
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      
      
      b2PolygonShape shape,shape1;
      shape.SetAsBox(4.0f, 0.7f);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->SetGravityScale(-1);
      
      body->CreateFixture(fd);
   

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = sbody;
      jointDef.localAnchorA.Set(3.8,0);
      jointDef.localAnchorB.Set(-1.0,1.0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
      
     }
      {
        b2PolygonShape shape0,shape1,shape2,shape3,shape4,shape5;
        b2BodyDef bd,bd1,bd2;
        //bd.type = b2_dynamicBody;
        bd.position.Set(7.5f, 41.0f);
        bd1.position.Set(18.52f, 38.2f);
        bd2.position.Set(29.185f, 35.6f);
        bd.type=b2_dynamicBody;
        bd1.type=b2_dynamicBody;
        bd2.type=b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2Body* body1 = m_world->CreateBody(&bd1);
        b2Body* body2 = m_world->CreateBody(&bd2);
        b2FixtureDef *fd0 = new b2FixtureDef;
        b2FixtureDef *fd1 = new b2FixtureDef;
        b2FixtureDef *fd2 = new b2FixtureDef;
        b2FixtureDef *fd3 = new b2FixtureDef;
        b2FixtureDef *fd4 = new b2FixtureDef;
        fd0->density = 0.63625f*20;
        fd1->density = 0.63625f*20;
        fd2->density = 1.00f*20;
        fd3->density = 0.63625f*20;
        fd4->density = 0.63625f*20;
        fd0->shape = new b2PolygonShape;
        fd1->shape = new b2PolygonShape;
        fd2->shape = new b2PolygonShape;
        fd3->shape = new b2PolygonShape;
        fd4->shape = new b2PolygonShape;
        fd0->shape = &shape0;
        fd1->shape = &shape1;
        fd2->shape = &shape2;
        fd3->shape = &shape3;
        fd4->shape = &shape4;
        shape0.SetAsBox(0.8f, 0.2f ,b2Vec2(-2.5,-3), 1.57f);
        shape1.SetAsBox(0.8f, 0.2f ,b2Vec2(-0.5,-3), 1.57f);
        shape2.SetAsBox(5.6f, 0.2f ,b2Vec2(5.5,1), 1.57f);
        shape3.SetAsBox(0.8f, 0.2f ,b2Vec2(-1.5,-3.8), 0);
        shape4.SetAsBox(3.f, 0.2f ,b2Vec2(2.75,-2.5), 0);
        body->CreateFixture(fd0);
        body->CreateFixture(fd1);
        body->CreateFixture(fd2);
        body->CreateFixture(fd3);
        body->CreateFixture(fd4);
        body1->CreateFixture(fd0);
        body1->CreateFixture(fd1);
        body1->CreateFixture(fd2);
        body1->CreateFixture(fd3);
        body1->CreateFixture(fd4);
        body2->CreateFixture(fd0);
        body2->CreateFixture(fd1);
        body2->CreateFixture(fd2);
        body2->CreateFixture(fd3);
        body2->CreateFixture(fd4);
        
 	    b2BodyDef hbd,hbd1,hbd2;
     	hbd.position.Set(5.75f, 38.f);
      	hbd1.position.Set(16.57f, 35.2f);
      	hbd2.position.Set(27.37f, 32.6f);
      	b2Body* hbody = m_world->CreateBody(&hbd);
      	b2Body* hbody1 = m_world->CreateBody(&hbd1);
      	b2Body* hbody2 = m_world->CreateBody(&hbd2);
      
      	b2PolygonShape hshape;
      
      	b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 50.0f;
      	fd->shape = new b2PolygonShape;
      	fd->shape = &hshape;
        hshape.SetAsBox(.4f, 0.2f ,b2Vec2(5.4,0.0), 0);
        hbody->CreateFixture(fd);
        hshape.SetAsBox(.4f, 0.2f ,b2Vec2(5.4,0.0), 0);
        hbody1->CreateFixture(fd);
        hshape.SetAsBox(.4f, 0.2f ,b2Vec2(5.4,0.0), 0);
        hbody2->CreateFixture(fd);
  
	 }
	 
	 {
  		b2BodyDef bd;
      bd.position.Set(40.f, 10.f);
      b2Body* body = m_world->CreateBody(&bd);
      
      //mid body
      b2PolygonShape shape1;
      shape1.SetAsBox(2.f, 2.5f, b2Vec2(0,-3.5),0);
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->shape = &shape1;
     
      //face
      
      b2BodyDef fac;
      fac.type = b2_dynamicBody;
      fac.position.Set(40.f, 10.f);
      b2Body* face = m_world->CreateBody(&fac);
      
      
      b2CircleShape shape2;
      shape2.m_radius = 1.0;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->shape = &shape2;
      
      face->CreateFixture(fd2);
      
      //legs
      b2PolygonShape shape3;
      shape3.SetAsBox(0.3f, 2.f, b2Vec2(-1.5,-7),0);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->shape = &shape3;
      
      b2PolygonShape shape4;
      shape4.SetAsBox(0.3f, 2.f, b2Vec2(1.5,-7),0);
      b2FixtureDef *fd4 = new b2FixtureDef;
      fd4->shape = &shape4;
      
      //hands
      b2PolygonShape shape5;
      shape5.SetAsBox(0.3f, 2.f, b2Vec2(2.5,-3),0.3);
      b2FixtureDef *fd5 = new b2FixtureDef;
      fd5->shape = &shape5;
      
      
      
      
      
      //fixtures
      body->CreateFixture(fd1);
      //body->CreateFixture(fd2);
      body->CreateFixture(fd3);
      body->CreateFixture(fd4);
      body->CreateFixture(fd5);
      //body->CreateFixture(fd6);
      
      
      //elbow
      
      
      
      
      
  }
  
  {
  	{
		b2BodyDef bd;
		bd.position.Set(38.3f, 27.0f);
		b2BodyDef bd1;
		bd1.position.Set(38.3f, 23.0f);
		b2BodyDef bd2;
		bd2.position.Set(39.3f, 19.5f);
		b2BodyDef bd3;
		bd3.position.Set(38.05f, 17.3f);
		b2Body* body = m_world->CreateBody(&bd); 
		b2Body* body1 = m_world->CreateBody(&bd1); 
		b2Body* body2 = m_world->CreateBody(&bd2); 
		b2Body* body3 = m_world->CreateBody(&bd3); 
		b2PolygonShape shape;
        
		b2FixtureDef *fd = new b2FixtureDef;
		fd->density = 50.0f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
		shape.SetAsBox(3.5,0.2,b2Vec2(1.0,0.5),1.57/2);
		body->CreateFixture(fd);
		shape.SetAsBox(3.0,0.2,b2Vec2(-2.0,0.0),1.57/2);
		body->CreateFixture(fd);
		
		shape.SetAsBox(3.3,0.2,b2Vec2(0.9,0),-1.57/2);
		body1->CreateFixture(fd);
		shape.SetAsBox(3.0,0.2,b2Vec2(-2.0,0.0),-1.57/2);
		body1->CreateFixture(fd);
		
		shape.SetAsBox(2.0,0.2,b2Vec2(0.9,0),1.57/2);
		body2->CreateFixture(fd);
		shape.SetAsBox(2.0,0.2,b2Vec2(-2.0,0.0),1.57/2);
		body2->CreateFixture(fd);
		
		shape.SetAsBox(1.0,0.2,b2Vec2(0.9,0),1.57);
		body3->CreateFixture(fd);
		shape.SetAsBox(1.0,0.2,b2Vec2(-2.0,0.0),1.57);
		body3->CreateFixture(fd);
		
     }
  }
  
  
  {
      b2PolygonShape shape;
      shape.SetAsBox(2.5f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(40.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(40.0f, 14.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(1.5,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
      
      b2PolygonShape shape_s;
      shape_s.SetAsBox(0.2f, 0.2f);
      fd->shape = new b2PolygonShape;
      fd->shape = &shape_s;
      fd->density = 43.87;
      
      b2BodyDef bd_s;
      bd_s.position.Set(40.5f, 15.0f);
      bd_s.type = b2_dynamicBody;
      b2Body* body_s = m_world->CreateBody(&bd_s);
      body_s->CreateFixture(fd);
      
    }
  
  
  
  //void checkAngle(b2Body* bd);
  
  
  
  }
  
  
  
  

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
  
  
}




