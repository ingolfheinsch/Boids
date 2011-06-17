#region usings
using System;
using System.Collections;
using System.ComponentModel.Composition;

using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;

using VVVV.Core.Logging;
#endregion usings

namespace VVVV.Nodes
{
	#region PluginInfo
	[PluginInfo(Name = "Boids", Category = "Animation", Version = "0.01", Help = "Basic template with one value in/out", Tags = "")]
	#endregion PluginInfo
	

	
	public class C0_01AnimationBoidsNode : IPluginEvaluate
	{
		#region fields & pins
		
		Flock flock = new Flock();
		
		int boidCount;
		
		int width;
	    int height;
	  	
	  	public static double cohe;
	  	public static double align;
	  	public static double desire;
	  	
	  	
	  	[Input("XRoot", DefaultValue = 0.0)]
		ISpread<double> FInput_XRoot;
		
		[Input("YRoot", DefaultValue = 0.0)]
		ISpread<double> FInput_YRoot;
		
		[Input("Cohesion", DefaultValue = 25.0)]
		ISpread<double> FInput_Cohesion;
		
		[Input("Align", DefaultValue = 25.0)]
		ISpread<double> FInput_Align;
		
		[Input("Desire", DefaultValue = 12.0)]
		ISpread<double> FInput_Desire;

		[Output("Output_X")]
		ISpread<double> FOutput_X;
		
		[Output("Output_Y")]
		ISpread<double> FOutput_Y;

		[Import()]
		ILogger FLogger;
		#endregion fields & pins
		
		 C0_01AnimationBoidsNode() {
		 
		 boidCount = 350;
		 width = 2;
		 height = 2;
		
		
		
		for (int i = 0; i < boidCount; i++) {
    		flock.addBoid(new Boid(new Vector2D(0,0), 1.2f, 0.04f));
  		}
  		

		}
		
		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
		
		cohe = FInput_Cohesion[0];
		align = FInput_Align[0];
		desire = FInput_Desire[0];
		
		
		
		 flock.run();
			FOutput_X.SliceCount = boidCount;
			FOutput_Y.SliceCount = boidCount;

			for (int i = 0; i < boidCount; i++){
				FOutput_X[i] = flock.getLoc(i).x;
				FOutput_Y[i] = flock.getLoc(i).y;
				
				}
			//FLogger.Log(LogType.Debug, "Logging to Renderer (TTY)");
		}
	}
	
	
	
	public class Boid
	{
		public Vector2D loc;
		Vector2D acc;
		Vector2D vel;
		float r;
		//maximum steering force
		float maxforce;
		//maximum steering speed
		float maxspeed;
		
		public Boid(Vector2D l, float ms, float mf)
		{
			acc = new Vector2D(0,0);
			vel = new Vector2D(-.5f,.5f);
			loc = l;
			maxspeed = ms;
			maxforce = mf;
		}
		
		public void run (ArrayList boids) {
		flock(boids);
		update();
		borders();
		}	
		
		void flock (ArrayList boids) {
		
		Vector2D sep = seperate(boids);
		Vector2D ali = align(boids);
		Vector2D coh = cohesion(boids);
		
		// Arbitrarily weight these forces
		sep *= 1.5;
		ali *= 1;
		coh *= 1;
		

	    // Add the force vectors to acceleration
	    acc+=sep;
	    acc+=ali;
	    acc+=coh;
		}
		
		void update() {
	    // Update velocity
	    vel+=acc;
	    // Limit speed
	    vel = limit (vel, maxspeed);
	    loc+=vel;
	    // Reset accelertion to 0 each cycle
	    acc*=0;
  		}
  		
  		void seek (Vector2D target) {
  		acc+=steer(target,false);
  		}
  		
  		void arrive(Vector2D target) {
		acc+=steer(target,true);
		}
		
		Vector2D steer (Vector2D target, bool slowdown) {
		
		Vector2D steer;
		Vector2D desired = target - loc;
		
		double d = !desired;
		
		 if (d > 0) {
      		// Normalize desired
      		desired = ~desired;
		// Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)
     	if ((slowdown) && (d < 100.0)) desired*=(maxspeed*(d/100.0)); // This damping is somewhat arbitrary
	      else desired*=maxspeed;
	      // Steering = Desired minus Velocity
	      steer = desired-vel;
	      steer =limit(steer, maxforce);  // Limit to maximum steering force
	      
	    } 
	    else {
	      steer = new Vector2D(0,0);
    	}
		return steer;
		}
		
		
		void borders() {
		if (loc.x < -r) loc.x = 400+r;
    	if (loc.y < -r) loc.y = 400+r;
   	 	if (loc.x > 400+r) loc.x = -r;
   		 if (loc.y > 400+r) loc.y = -r;
		}
		
		/*
		void borders() {
		if (loc.x < 0) vel.x *=-1;
    	if (loc.y < 0) vel.y *=-1;
   	 	if (loc.x > 400) vel.x *=-1;
   		 if (loc.y > 400) vel.y *=-1;
		}
		*/
		  // Separation
  // Method checks for nearby boids and steers away
  	Vector2D seperate (ArrayList boids) {
    	float desiredseparation = (float) C0_01AnimationBoidsNode.desire;
    	Vector2D steer = new Vector2D(0,0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (int i = 0 ; i < boids.Count; i++) {
      Boid other = (Boid) boids[i];
      float d = dist(loc,other.loc);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        Vector2D diff = loc-other.loc;
        diff = ~diff;
        diff/=d;        // Weight by distance
        steer+=diff;
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer/=(float)count;
    }

    // As long as the vector is greater than 0
    if (!steer > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer=~steer;
      steer*=maxspeed;
      steer-=vel;
      steer = limit(steer,maxforce);
      
    }
    return steer;
  }
  
  
    // Alignment
  // For every nearby boid in the system, calculate the average velocity
  Vector2D align (ArrayList boids) {
    float neighbordist = (float) C0_01AnimationBoidsNode.align;
    Vector2D steer = new Vector2D(0,0);
    int count = 0;
    for (int i = 0 ; i < boids.Count; i++) {
      Boid other = (Boid) boids[i];
      float d = dist(loc,other.loc);
      if ((d > 0) && (d < neighbordist)) {
        steer+=other.vel;
        count++;
      }
    }
    if (count > 0) {
      steer/=(float)count;
    }

    // As long as the vector is greater than 0
    if (!steer > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
  	steer=~steer;
      steer*=maxspeed;
      steer-=vel;
      steer = limit(steer,maxforce);
    }
    return steer;
  }
  
  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  Vector2D cohesion (ArrayList boids) {
    float neighbordist = (float) C0_01AnimationBoidsNode.cohe;
    Vector2D sum = new Vector2D(0,0);   // Start with empty vector to accumulate all locations
    int count = 0;
    for (int i = 0 ; i < boids.Count; i++) {
      Boid other = (Boid) boids[i];
      float d = dist(loc,other.loc);
      if ((d > 0) && (d < neighbordist)) {
        sum+=other.loc; // Add location
        count++;
      }
    }
    if (count > 0) {
      sum/=(float)count;
      return steer(sum,false);  // Steer towards the location
    }
    return sum;
  }

		
	public Vector2D limit(Vector2D v, float max) {
    if (!v > max) {
      v = ~v;
     v*= max;
    }
    return v;
  }
  
   public float dist(Vector2D v0, Vector2D v1) {
 	 double dx = v0.x - v1.x;
    double dy = v0.y - v1.y;
    //float dz = v1.z - v2.z;
    return (float) Math.Sqrt(dx*dx + dy*dy );

  }

		


	}
	
	public class Flock
	{
	ArrayList boids;
	
	public Flock() {
	boids = new ArrayList();
	}
	
	public void run() {
    for (int i = 0; i < boids.Count; i++) {
      Boid b = (Boid) boids[i];  
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }
  public Vector2D getLoc(int index) {
 	Boid b = (Boid) boids[index];
 	return b.loc;
  }
  
   public void addBoid(Boid b) {
    boids.Add(b);
  }


	
	}
	
}
