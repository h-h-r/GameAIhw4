using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This is the place to put all of the various steering behavior methods we're going
/// to be using. Probably best to put them all here, not in NPCController.
/// </summary>

public class SteeringBehavior : MonoBehaviour {

    // The agent at hand here, and whatever target it is dealing with
    public NPCController agent;
    public NPCController target;
    public NPCController red;

    // Below are a bunch of variable declarations that will be used for the next few
    // assignments. Only a few of them are needed for the first assignment.

    // For pursue and evade functions
    public float maxPrediction;
    public float maxAcceleration;

    // For arrive function
    public float maxSpeed;
    public float targetRadiusL;
    public float slowRadiusL;
    public float timeToTarget;

    // For Face function
    public float maxRotation;
    public float maxAngularAcceleration;
    public float targetRadiusA;
    public float slowRadiusA;

    // For wander function
    public float wanderOffset;
    public float wanderRadius;
    public float wanderRate;
    private float wanderOrientation;

    // Holds the path to follow
    public GameObject[] Path;
    public int current = 0;
    public GameObject[] WolfPath;
    public GameObject[] HunterPath;

    public int separationWeight = 3;
    public int alignmentWeight = 2;
    public int cohesionWeight = 1;

    public float separationRadius = 1.5f;
    

    protected void Start() {
        agent = GetComponent<NPCController>();
        wanderOrientation = agent.orientation;
        red = GameObject.FindGameObjectWithTag("Red").GetComponent<NPCController>();
    }

    
    public Vector3 SeparationAcc()
    {

        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        Collider[] group = Physics.OverlapSphere(agent.position, separationRadius);
        //print(group.Length + "=> length\n");
        for (int i = 0; i < group.Length; i++)
        {
            if (group[i].tag == "Wolf" || group[i].tag == "Red" || group[i].tag == "Hunter" )
            {
                //Debug.Log("trans:<" + group[i].transform.position + "> npc<" + group[i].GetComponent<NPCController>().position+">\n");
                Vector3 direction = agent.position - group[i].GetComponent<NPCController>().position;
                if (direction.magnitude > 0.0001f)
                {
                    //group[i].GetComponent<SteeringBehavior>().agent.DrawCircle(agent.position, 0.5f);
                    linear_acc += direction / direction.magnitude / direction.magnitude;
                }

                if (group[i].tag == "Red")
                {
                    linear_acc += 2 * direction / direction.magnitude / direction.magnitude;
                }

            }

            //print(">>>" + group[i].tag + ".\n");
        }
        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        //print("linear acc = " + linear_acc + "\n");
        return linear_acc;
    }

    public Vector3 SeparationAccWithPrediction()
    {

        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        Collider[] group = Physics.OverlapSphere(agent.position, separationRadius);
        //print(group.Length + "=> length\n");
        for (int i = 0; i < group.Length; i++)
        {
            if (group[i].tag == "Wolf" || group[i].tag == "Red" || group[i].tag == "Hunter")
            {
                //Debug.Log("trans:<" + group[i].transform.position + "> npc<" + group[i].GetComponent<NPCController>().position+">\n");
                //Vector3 
                Vector3 direction;
                if (agent.tag != group[i].tag)
                {
                    //Debug.Log
                    direction = Evade(group[i].GetComponent<NPCController>());
                    linear_acc +=  5 * direction / direction.magnitude / direction.magnitude;
                }
                else
                {
                    direction = agent.position - group[i].GetComponent<NPCController>().position;
                }
               
                if (direction.magnitude > 0.0001f)
                {
                    //group[i].GetComponent<SteeringBehavior>().agent.DrawCircle(agent.position, 0.5f);
                    linear_acc += direction / direction.magnitude / direction.magnitude;
                }

                if (group[i].tag == "Red")
                {
                    linear_acc += 2 * direction / direction.magnitude / direction.magnitude;
                }

            }

            //print(">>>" + group[i].tag + ".\n");
        }
        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        //print("linear acc = " + linear_acc + "\n");
        return linear_acc;
    }


    public Vector3 CohesionAcc(NPCController leader)
    {
        //if (leader != null)
        //{
        //    return Arrive(leader.position - agent.position);
        //}
        

        //Vector3 linear_acc = new Vector3(0f, 0f, 0f);

        //return linear_acc;
        GameObject[] group = GameObject.FindGameObjectsWithTag(agent.tag);
        Vector3 avgPosition = new Vector3(0f, 0f, 0f);
        for (int i = 0; i < group.Length; i++)
        {
            avgPosition += group[i].GetComponent<NPCController>().position;
        }

        if (leader != null)
        {
            avgPosition += 2 * group.Length * leader.position;
            avgPosition /= (group.Length * 3);
        }
        else
        {
            avgPosition /= group.Length;
        }
        
        return Arrive(avgPosition - agent.position);

    }

    public Vector3 Align(NPCController leader)
    {

        //GameObject[] group = GameObject.FindGameObjectsWithTag(tag);
        Collider[] group = Physics.OverlapSphere(agent.position, 4f);
        //print(group.Length+">>>\n");

        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        Vector3 avg_velocity = new Vector3(0f, 0f, 0f);
        int count = 0;

        for (int i = 0; i < group.Length; i++)
        {
            if (group[i].tag == agent.tag)
            {
                Vector3 direction = agent.position - group[i].GetComponent<NPCController>().position;
                //if agent with group[i] too close
                if (direction.magnitude > 0.00001)
                {
                    avg_velocity += group[i].GetComponent<NPCController>().velocity;
                    count += 1;
                    //linear_acc += direction / direction.magnitude / direction.magnitude;
                    //print("i:" + i + ", linearacc " + linear_acc + "\n");
                }
            }
        }
        //if have leader
        if (leader != null)
        {
            if ((leader.position - agent.position).magnitude < 4f)
            {
                avg_velocity += leader.velocity;
                count += 1;
            }
        }
       

        if (count == 0)
        {
            return linear_acc;
        }
        else
        {
            avg_velocity /= count;
            linear_acc = (avg_velocity - agent.velocity).normalized * maxAcceleration;
            return linear_acc;
        }
        
    }
    

    public Vector3 FlockingAccWithLeader(NPCController red)
    {
        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        linear_acc += this.separationWeight * SeparationAcc();
        linear_acc += this.alignmentWeight * Align(red);
        linear_acc += this.cohesionWeight * CohesionAcc(red);
        
        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        //print(linear_acc + ">>\n");
        return linear_acc;
    }

    //expect target is leader
    public Vector3 FlockingAccWithLeader()
    {
        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        //linear_acc += this.separationWeight * SeparationAcc();
        linear_acc += this.separationWeight * SeparationAccWithPrediction();
        linear_acc += this.alignmentWeight * Align(this.target);
        linear_acc += this.cohesionWeight * CohesionAcc(this.target);

        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        //print(linear_acc + ">>\n");
        return linear_acc;
    }

    public Vector3 FlockingAccWithPathFollowing()
    {
        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        linear_acc += this.separationWeight * SeparationAcc();
        linear_acc += this.alignmentWeight * Align(null);
        linear_acc += this.cohesionWeight * CohesionAcc(null);

        linear_acc += 6 * this.PathFollow().Item1;

        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        //print(linear_acc + ">>\n");
        
        return linear_acc;
    }


    // ETC. ===================================================previuos algo (Some maybe slightly modified)
    //Haoran
    //evade from target
    //return linear acc
    public Vector3 Evade()
    {
        float distance = (target.position - agent.position).magnitude;

        // speed scalar
        float speed = agent.velocity.magnitude;

        // if speed small-> use bigger predictionTime
        float prediction = (speed <= distance / maxPrediction ? maxPrediction : distance / speed);

        //draw prediction circle
        agent.DrawCircle(target.position + target.velocity * prediction, 0.5f);

        //direction to evade prediction point
        Vector3 linear_acc = agent.position - (target.position + target.velocity * prediction);

        //clip to map linear acc
        linear_acc.Normalize();
        linear_acc *= maxAcceleration;

        return linear_acc;
    }

    public Vector3 Evade(NPCController target)
    {
        float distance = (target.position - agent.position).magnitude;

        // speed scalar
        float speed = agent.velocity.magnitude;

        // if speed small-> use bigger predictionTime
        float prediction = (speed <= distance / maxPrediction ? maxPrediction : distance / speed);

        //draw prediction circle
        agent.DrawCircle(target.position + target.velocity * prediction, 0.5f);

        //direction to evade prediction point
        Vector3 linear_acc = agent.position - (target.position + target.velocity * prediction);

        //clip to map linear acc
        linear_acc.Normalize();
        linear_acc *= maxAcceleration;

        return linear_acc;
    }

    //Haoran
    //pathfollow
    public (Vector3, float) PathFollow()
    {
        Vector3 targetOnPath = new Vector3(0,0,0);
        if (agent.tag == "Wolf")
        {
            //Debug.Log(">>>" + this.WolfPath.Length);
            targetOnPath = computeTargetOnThePath(this.WolfPath);
        }
        else if (agent.tag == "Hunter")
        {
            //Debug.Log(">>" + this.WolfPath.Length);
            targetOnPath = computeTargetOnThePath(this.HunterPath);
        }
       
        //Debug.Log(targetOnPath);
        if (target == null)
        {
            agent.DrawCircle(targetOnPath, 0.3f);
        }
        
        Vector3 linearAcc = Seek(targetOnPath);
        float angularAcc = FaceTo(targetOnPath - agent.position);
        return (linearAcc, angularAcc);
    }

    //Haoran
    //pathfollow helper function: find the target position on path
    private Vector3 computeTargetOnThePath(GameObject[] Path)
    {
        //compute line vectors,  n nodes => n-1 vectors
        List<Vector3> lineVectors = new List<Vector3>();
        for (int i = 1; i < Path.Length; i++)
        {
            lineVectors.Add(Path[i].transform.position - Path[i - 1].transform.position);
            //Debug.Log(i + "$" + lineVectors[i - 1]);
        }
        //Debug.Log(agent.position);
        //compute projected points, n nodes => n-1 vectors => n-1 prjected points 
        List<Vector3> projectedPoints = new List<Vector3>();
        for (int i = 0; i < lineVectors.Count; i++)
        {
            //Debug.Log(i+"?"+ (Path[i].transform.position + Vector3.Project(agent.position - Path[i].transform.position, lineVectors[i])));
            //projectedPoints.Add(Path[i].transform.position + Vector3.Project(agent.position- Path[i].transform.position, lineVectors[i] ));
            //agent.DrawCircle(Path[i].transform.position + Vector3.Project(agent.position - Path[i].transform.position, lineVectors[i]),0.5f);
            projectedPoints.Add(FindNearestPointOnLine(Path[i].transform.position, Path[i + 1].transform.position, agent.position));
            //Debug.Log(i + "?" + FindNearestPointOnLine(Path[i].transform.position, Path[i + 1].transform.position, agent.position));



        }

        //find nearest projection point
        int index = 0;
        Vector3 nearestPoint = new Vector3(0, 0, 0);
        float distance = Mathf.Infinity;

        List<float> distList = new List<float>();


        for (int i = 0; i < projectedPoints.Count; i++)
        {
            if ((agent.position - projectedPoints[i]).magnitude < distance)
            {
                distance = (agent.position - projectedPoints[i]).magnitude;
                index = i;
                nearestPoint = projectedPoints[i];
            }
        }

        //Debug.Log("Nearest Index = " + index + ".");
        //agent.DrawCircle(nearestPoint, 0.3f);

        //calculate the target to seek
        //index 0 ~ n-1
        float lookAheadDistance = 2;
        Vector3 targetPoint = new Vector3(0, 0, 0);

        //Debug.Log
        //Debug.Log(index + ">index\n");
        //Debug.Log(Path[index + 1] + ">p index+1\n");

        if ((Path[Path.Length-1].transform.position - projectedPoints[index]).magnitude < lookAheadDistance)
        {
            return Path[Path.Length - 1].transform.position;
        }

        if (lookAheadDistance < (Path[index + 1].transform.position - projectedPoints[index]).magnitude)
        {
            targetPoint = projectedPoints[index] + lookAheadDistance * (lineVectors[index].normalized);
        }
        else
        {
            lookAheadDistance -= (Path[index + 1].transform.position - projectedPoints[index]).magnitude;
            for (int k = index + 1; k < lineVectors.Count; k++)
            {
                if (lineVectors[k].magnitude < lookAheadDistance)
                {
                    lookAheadDistance -= lineVectors[k].magnitude;
                }
                else
                {
                    targetPoint = projectedPoints[k] + lookAheadDistance * (lineVectors[k].normalized);
                    //check last segment
                    if (k == lineVectors.Count - 1 && lookAheadDistance > lineVectors[k].magnitude)
                    {
                        targetPoint = Path[k + 1].transform.position;
                    }
                    break;
                }
            }

        }

        return targetPoint;

    }

    //Haoran
    // find nearest position on line segment
    private Vector3 FindNearestPointOnLine(Vector3 origin, Vector3 end, Vector3 point)
    {
        //Get heading
        Vector3 heading = (end - origin);
        float magnitudeMax = heading.magnitude;
        heading.Normalize();

        //Do projection from the point but clamp it
        Vector3 lhs = point - origin;
        float dotP = Vector3.Dot(lhs, heading);
        dotP = Mathf.Clamp(dotP, 0f, magnitudeMax);
        return origin + heading * dotP;
    }

    //Haoran
    //seek target
    //return linear acc
    public Vector3 Seek()
    {
        return Seek(target.position);
    }
    public Vector3 Seek(Vector3 targetPosition)
    {
        Vector3 linear_acc = targetPosition - agent.position; //seek direction vector

        //clip to max linear acceleration
        if (linear_acc.magnitude > this.maxAcceleration)
        {
            linear_acc = linear_acc.normalized * maxAcceleration;
        }

        //clip to max speed is handled in the UpdateMovement in NPCController.cs 
        //angular acceleration will be handled by face()  

        return linear_acc;  //returns the linear acc 
    }

    //Haoran
    //dynamic arrive given target
    //return linear acc
    public Vector3 Arrive()
    {
        Vector3 direction = target.position - agent.position;
        return Arrive(direction);
    }

    //Haoran
    //dynamic arrive given direction
    //return linear acc
    public Vector3 Arrive(Vector3 direction)
    {
        // Vector3 direction = target.position - agent.position;

        if (direction.magnitude <= slowRadiusL)
        {
            //agent.label.text = "dynamic arrive\n<In slowRadiusL>";
            //agent.DestroyPoints();
            //agent.DrawCircle(target.position, slowRadiusL);
        }
        else
        {
            agent.DestroyPoints();
        }

        // stop if arrive (in target radius) and return zero linear_acc
        if (direction.magnitude < targetRadiusA )
        {
            //agent.label.text = "dynamic arrive\n<In targetRadiusA>";
            agent.velocity = Vector3.zero;
            return Vector3.zero;
        }

        //calculate appropriate speed
        float speed = (direction.magnitude > slowRadiusL ? maxSpeed : maxSpeed * direction.magnitude / slowRadiusL);

        //apply direction
        direction.Normalize();
        Vector3 velocity = direction * speed;

        // calculate linear_acc
        Vector3 linear_acc = (velocity - agent.velocity) / timeToTarget;

        // clip linear_acc
        if (linear_acc.magnitude > maxAcceleration)
        {
            linear_acc.Normalize();
            linear_acc *= maxAcceleration;
        }

        return linear_acc;
    }

    //Haoran
    // Calculate the angular acceleration required to rotate to target
    //return angular acc
    public float Face()
    {
        Vector3 direction = target.position - agent.position;
        return FaceTo(direction);
    }

    //Haoran
    //face to given direction
    //return angular acc
    public float FaceTo(Vector3 direction)
    {
        // Check for a zero direction, and make no change if so
        if (direction.magnitude == 0)
        {
            return 0;
        }

        // Get anount of angle need to rotate
        float rotationAmount = Mathf.Atan2(direction.x, direction.z) - agent.orientation;
        //agent.orientaion range [-inf,inf]

        // clip to (-pi, pi) interval
        while (rotationAmount > Mathf.PI)
        {
            rotationAmount -= 2 * Mathf.PI;
        }
        while (rotationAmount < -Mathf.PI)
        {
            rotationAmount += 2 * Mathf.PI;
        }

        // if already facing target, set angular speed to zero
        if (Mathf.Abs(rotationAmount) < targetRadiusA)
        {
            agent.rotation = 0;
        }

        // greater than slowRadius => clip to max rotation speed
        // less than slowRadius => clip to scaled rotation speed 
        float rotationSpeed = (rotationAmount > slowRadiusA ? maxRotation : maxRotation * Mathf.Abs(rotationAmount) / slowRadiusA);

        // get the correct rotation direction
        rotationSpeed *= rotationAmount / Mathf.Abs(rotationAmount);

        // calculate the rotation acceleration
        float angular_acc = rotationSpeed - agent.rotation;
        angular_acc /= timeToTarget;

        // clip to max angular acc if needed
        if (Mathf.Abs(angular_acc) > maxAngularAcceleration)
        {
            angular_acc /= Mathf.Abs(angular_acc);
            angular_acc *= maxAngularAcceleration;
        }

        return angular_acc;
    }


}
