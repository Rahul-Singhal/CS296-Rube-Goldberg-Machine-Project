
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
{
  dominos_t::dominos_t()
  {
    //Ground
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    
      
    

    //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->position.Set(-3.0,16);
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
      //box1->CreateFixture(fd1);

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
      //b2Vec2 worldAnchorOnBody1(-35, 17); // Anchor point on body 1 in world axis
      //b2Vec2 worldAnchorOnBody2(-31, 17); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-35, 22); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(-31, 22); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, b2Vec2(-35,10), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
      
      
      
      
      
      
      
      
      
      
  		b2PolygonShape shape6;
      shape6.SetAsBox(0.3f, 1.f,b2Vec2(0,0),-0.5);
      b2FixtureDef *fd6 = new b2FixtureDef;
      fd6->shape = &shape6;
      fd6->density = 50.0f;
      
      b2BodyDef should;
      should.position.Set(37.5f, 8.f);
      b2Body* shoulder = m_world->CreateBody(&should);
      shoulder->CreateFixture(fd6);
      
      
      b2PolygonShape shape7;
      shape7.SetAsBox(0.3f, 1.5f,b2Vec2(-0.3,-1.2),-0.4);
      b2FixtureDef *fd7 = new b2FixtureDef;
      fd7->shape = &shape7;
      fd7->density = 50.0f;
      
      
      b2BodyDef elb;
      elb.type = b2_dynamicBody;
      elb.position.Set(37.f, 7.f);
      b2Body* elbow = m_world->CreateBody(&elb);
      elbow->SetGravityScale(-8);
      //elbow->SetAngularVelocity(-1.0);
      elbow->CreateFixture(fd7);
      
    
      
      
      
      
      
      b2RevoluteJointDef elbowJoint;
      elbowJoint.bodyA = shoulder;
      elbowJoint.referenceAngle = 2;
      elbowJoint.bodyB = elbow;
      elbowJoint.localAnchorA.Set(-0.5,-0.7);
      elbowJoint.localAnchorB.Set(0,0);
      elbowJoint.collideConnected = false;
      
      //elbowJoint.enableMotor = true;
			//elbowJoint.maxMotorTorque = 20;
			//elbowJoint.motorSpeed = 50;
			m_world->CreateJoint(&elbowJoint);
			
			b2PulleyJointDef* joint = new b2PulleyJointDef();
     	b2Vec2 worldAnchorOnBody11(36.5, 5); // Anchor point on body 1 in world axis
      //b2Vec2 worldAnchorOnBody21(-35, 17); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround11(37.5, 2); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround21(-35, 2); // Anchor point for ground 2 in world axis
      //float32 ratio = 1.0f; // Define ratio
      joint->Initialize(elbow, box1, worldAnchorGround11, worldAnchorGround21, worldAnchorOnBody11, b2Vec2(-35,10), 1.0f);
      m_world->CreateJoint(joint);
      
  
      
      
      
      
      
      
      
      
    }
    
    {
    	//the platform with balls on it
      
      b2BodyDef bd;
      bd.position.Set(-26.0f, 18.0f);
      b2Body* body = m_world->CreateBody(&bd);
      
      b2PolygonShape shape;
      shape.SetAsBox(4.f, 0.2f);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      
      bd.position.Set(-11.5f, 14.0f);
      body = m_world->CreateBody(&bd);
      body->CreateFixture(fd);
      
      bd.position.Set(3.f, 10.0f);
      body = m_world->CreateBody(&bd);
      body->CreateFixture(fd);
    }
    
    {
    	b2BodyDef bd;
      bd.position.Set(-18.5f, 15.8f);
      b2Body* body = m_world->CreateBody(&bd);
      
      b2PolygonShape shape;
      
      b2Vec2 center;
      center.Set(0.f,0.f);
      shape.SetAsBox(4.f, 0.2f,center , -10);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      
      bd.position.Set(-4.2f, 11.8f);
      body = m_world->CreateBody(&bd);
      body->CreateFixture(fd);
    }
    
    {
    	//the balls on the platform
    	
    	b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 20.0f;
      ballfd.friction = 0.2f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-29.5f, 19.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      
      ballbd.position.Set(5.5f, 11.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      
      ballbd.position.Set(-8.0f, 15.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      
    }
    
    {
      b2PolygonShape shape;
      shape.SetAsBox(8.2f, 0.2f);
      b2PolygonShape shape1;
      shape1.SetAsBox(0.2f, 1.0f , b2Vec2(8.f,1.f),0);
	
      b2BodyDef bd;
      bd.position.Set(18.0f, 4.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      
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

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(-2,0);
      jointDef.localAnchorB.Set(-2,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
      
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.5f;
      //ballfd.friction = 0.0f;
      //ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(20.0f, 5.4f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      
      b2PolygonShape shape3;
      shape3.SetAsBox(0.2f, 0.2f);
      b2BodyDef bd3;
      bd3.position.Set(21.0f, 3.58f);
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->shape = &shape3;
      body3->CreateFixture(fd3);
      
    }
    
    {
    	//second level
    	
    	b2BodyDef bd;
      bd.position.Set(0.0f, 24.0f);
      b2Body* body = m_world->CreateBody(&bd);
      
      b2PolygonShape shape;
      shape.SetAsBox(4.f, 0.2f);
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 50.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      
     
      shape.SetAsBox(4.f, 0.2f ,b2Vec2(-7.6,-2), -5.8);
      body->CreateFixture(fd);
      
      }
    
 {
    
    	//the joint on the second level
    	
      
      b2PolygonShape shape2;
      shape2.SetAsBox(2.f, 0.2f);
      
      
      b2BodyDef bd2;
      bd2.position.Set(-12.6f, 20.2f);
      b2Body* body2 = m_world->CreateBody(&bd2);
      
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 20.0f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape2;
      
      
    	
    	//body2->CreateFixture(fd1);
      body2->CreateFixture(fd2);
      
      
      
      
    	
    	b2PolygonShape shape1;
      shape1.SetAsBox(4.f, 0.2f,b2Vec2(0,6), 0 );
      
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &shape1;
    	
    	
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
      
     
      

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,-4);
      jointDef.localAnchorB.Set(-0.4,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }
    
  

     
    
  
  }
  
  
  
  

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
  
  
}




