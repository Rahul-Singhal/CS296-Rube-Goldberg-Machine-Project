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
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  dominos_t::dominos_t()
  {
    //Ground
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	
      b2BodyDef bd;
      
      b2FixtureDef *fd1 = new b2FixtureDef;
			fd1->density = 50;
		  fd1->friction = 0.5;
		  fd1->restitution = 0.f;
		  fd1->shape = &shape;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(fd1);
    }
    
      
    

    //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->position.Set(-3.0,12);
      bd->type = b2_dynamicBody;
      bd->fixedRotation = true;
      


				b2FixtureDef *fd1 = new b2FixtureDef;
				fd1->density = 50;
		    fd1->friction = 0.5;
		    fd1->restitution = 0.f;
		    fd1->shape = new b2CircleShape;
		    b2CircleShape circ1;
		    circ1.m_p.Set(-32.f, 0.2f);
		    circ1.m_radius = 2.f;
				fd1->shape = &circ1;
			b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);

      //The bar{}
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
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-35, 17); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(-31, 17); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-35, 22); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(-31, 22); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    
    {
    	//the platform with balls on it
      
      
      b2EdgeShape shape;
      shape.Set(b2Vec2(-30.0f, 18.f), b2Vec2(-22.0f, 18.0f));
	
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
      
      shape.Set(b2Vec2(-22.f, 18.0f), b2Vec2(-15.5f, 14.0f));
      b1->CreateFixture(&shape, 0.0f);
      
      shape.Set(b2Vec2(-15.5f, 14.f), b2Vec2(-7.5f, 14.0f));
      b1->CreateFixture(&shape, 0.0f);
      
      shape.Set(b2Vec2(-7.5f, 14.0f), b2Vec2(0.0f, 10.0f));
      b1->CreateFixture(&shape, 0.0f);
      
      shape.Set(b2Vec2(0.0f, 10.f), b2Vec2(6.0f, 10.0f));
      b1->CreateFixture(&shape, 0.0f);
      
      
      //
      
      shape.Set(b2Vec2(4.0f, 24.f), b2Vec2(-4.0f, 24.0f));
      b1->CreateFixture(&shape, 0.0f);
      
      shape.Set(b2Vec2(-4.0f, 24.f), b2Vec2(-11.3f, 20.0f));
      b1->CreateFixture(&shape, 0.0f);
      
     	////
     	
     	shape.Set(b2Vec2(-18.0f, 26.f), b2Vec2(-32.0f, 26.0f));
      b1->CreateFixture(&shape, 0.0f);
     	
     	shape.Set(b2Vec2(-32.0f, 26.f), b2Vec2(-38.0f, 23.0f));
      b1->CreateFixture(&shape, 0.0f);
     	
     	shape.Set(b2Vec2(-38.0f, 23.f), b2Vec2(-48.0f, 23.0f));
      b1->CreateFixture(&shape, 0.0f);
      
    }
    
   
		
		

   
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
