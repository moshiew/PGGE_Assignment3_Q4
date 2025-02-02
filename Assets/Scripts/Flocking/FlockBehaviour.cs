using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;

[BurstCompile]
public class FlockBehaviour : MonoBehaviour
{
    List<Obstacle> mObstacles = new List<Obstacle>();

    [SerializeField]
    GameObject[] Obstacles;

    [SerializeField]
    BoxCollider2D Bounds;

    public float TickDuration = 1.0f;
    public float TickDurationSeparationEnemy = 0.1f;
    public float TickDurationRandom = 1.0f;

    public int BoidIncr = 100;
    public bool useFlocking = false;
    public int BatchSize = 100;

    public List<Flock> flocks = new List<Flock>();

    private NativeArray<float3> positions;
    private NativeArray<float3> velocities;
    private NativeArray<float3> newPositions;
    private NativeArray<float3> newVelocities;

    void Reset()
    {
        flocks = new List<Flock>()
        {
            new Flock()
        };
    }

    void Start()
    {
        // Randomize obstacles placement.
        for (int i = 0; i < Obstacles.Length; ++i)
        {
            float x = UnityEngine.Random.Range(Bounds.bounds.min.x, Bounds.bounds.max.x);
            float y = UnityEngine.Random.Range(Bounds.bounds.min.y, Bounds.bounds.max.y);
            Obstacles[i].transform.position = new Vector3(x, y, 0.0f);
            Obstacle obs = Obstacles[i].AddComponent<Obstacle>();
            Autonomous autono = Obstacles[i].AddComponent<Autonomous>();
            autono.MaxSpeed = 1.0f;
            obs.mCollider = Obstacles[i].GetComponent<CircleCollider2D>();
            mObstacles.Add(obs);
        }

        foreach (Flock flock in flocks)
        {
            CreateFlock(flock);
        }
    }

    void CreateFlock(Flock flock)
    {
        for (int i = 0; i < flock.numBoids; ++i)
        {
            float x = UnityEngine.Random.Range(Bounds.bounds.min.x, Bounds.bounds.max.x);
            float y = UnityEngine.Random.Range(Bounds.bounds.min.y, Bounds.bounds.max.y);

            AddBoid(x, y, flock);
        }
    }

    void Update()
    {
        HandleInputs();
        CalculateFlockMovement();
        ApplyMovement();
    }

    void CalculateFlockMovement()
    {
        // Create a job to process flock movement in parallel
        FlockJob flockJob = new FlockJob
        {
            Positions = positions,
            Velocities = velocities,
            NewPositions = newPositions,
            NewVelocities = newVelocities,
            BoundsMin = Bounds.bounds.min,
            BoundsMax = Bounds.bounds.max,
            Visibility = 10f,
            SeparationDistance = 2f,
            WeightAlignment = 1f,
            WeightSeparation = 1f,
            WeightCohesion = 1f
        };

        JobHandle jobHandle = flockJob.Schedule(positions.Length, BatchSize);
        jobHandle.Complete();

        // Swap buffers to update positions and velocities after the job finishes
        NativeArray<float3> tempPositions = positions;
        positions = newPositions;
        newPositions = tempPositions;

        NativeArray<float3> tempVelocities = velocities;
        velocities = newVelocities;
        newVelocities = tempVelocities;
    }

    void ApplyMovement()
    {
        foreach (Flock flock in flocks)
        {
            List<Autonomous> validBoids = new List<Autonomous>(flock.mAutonomous);

            foreach (Autonomous boid in validBoids)
            {
                if (boid != null)
                {
                    Vector3 randomWander = new Vector3(
                        UnityEngine.Random.Range(-0.05f, 0.05f),
                        UnityEngine.Random.Range(-0.05f, 0.05f),
                        0);

                    boid.TargetDirection += randomWander;
                    boid.TargetDirection.Normalize();
                    boid.transform.position += boid.TargetDirection * Time.deltaTime;
                }
            }
        }
    }

    void Execute(Flock flock, int i)
    {
        Vector3 flockDir = Vector3.zero;
        Vector3 separationDir = Vector3.zero;
        Vector3 cohesionDir = Vector3.zero;

        float speed = 0.0f;
        float separationSpeed = 0.0f;

        int count = 0;
        int separationCount = 0;
        Vector3 steerPos = Vector3.zero;

        Autonomous curr = flock.mAutonomous[i];

        Vector3 randomDirection = new Vector3(
            UnityEngine.Random.Range(-0.05f, 0.05f),
            UnityEngine.Random.Range(-0.05f, 0.05f),
            0);

        for (int j = 0; j < flock.numBoids; ++j)
        {
            Autonomous other = flock.mAutonomous[j];

            float dist = (curr.transform.position - other.transform.position).magnitude;

            if (i != j && dist < flock.visibility)
            {
                speed += other.Speed;
                flockDir += other.TargetDirection;
                steerPos += other.transform.position;
                count++;
            }

            if (i != j && dist < flock.separationDistance)
            {
                Vector3 targetDirection = (curr.transform.position - other.transform.position).normalized;
                separationDir += targetDirection;
                separationSpeed += dist * flock.weightSeparation;
                separationCount++;
            }
        }

        if (count > 0)
        {
            speed = speed / count;
            flockDir = flockDir / count;
            flockDir.Normalize();
            steerPos = steerPos / count;
        }

        if (separationCount > 0)
        {
            separationSpeed = separationSpeed / count;
            separationDir = separationDir / separationSpeed;
            separationDir.Normalize();
        }

        curr.TargetDirection =
            flockDir * speed * (flock.useAlignmentRule ? flock.weightAlignment : 0.0f) +
            separationDir * separationSpeed * (flock.useSeparationRule ? flock.weightSeparation : 0.0f) +
            (steerPos - curr.transform.position) * (flock.useCohesionRule ? flock.weightCohesion : 0.0f) +
            randomDirection;
    }

    void HandleInputs()
    {
        if (EventSystem.current.IsPointerOverGameObject() || !enabled)
        {
            return;
        }

        if (Input.GetKeyDown(KeyCode.Space))
        {
            AddBoids(BoidIncr);
        }
    }

    void AddBoids(int count)
    {
        for (int i = 0; i < count; ++i)
        {
            float x = UnityEngine.Random.Range(Bounds.bounds.min.x, Bounds.bounds.max.x);
            float y = UnityEngine.Random.Range(Bounds.bounds.min.y, Bounds.bounds.max.y);

            AddBoid(x, y, flocks[0]);
        }
        flocks[0].numBoids += count;
    }

    void AddBoid(float x, float y, Flock flock)
    {
        GameObject obj = Instantiate(flock.PrefabBoid);
        obj.name = "Boid_" + flock.name + "_" + flock.mAutonomous.Count;
        obj.transform.position = new Vector3(x, y, 0.0f);
        Autonomous boid = obj.GetComponent<Autonomous>();
        flock.mAutonomous.Add(boid);
        boid.MaxSpeed = flock.maxSpeed;
        boid.RotationSpeed = flock.maxRotationSpeed;

        boid.TargetDirection = new Vector3(
            UnityEngine.Random.Range(-1f, 1f),
            UnityEngine.Random.Range(-1f, 1f),
            0).normalized;
    }

    static float Distance(Autonomous a1, Autonomous a2)
    {
        return (a1.transform.position - a2.transform.position).magnitude;
    }

    void SeparationWithEnemies_Internal(
        List<Autonomous> boids,
        List<Autonomous> enemies,
        float sepDist,
        float sepWeight)
    {
        for (int i = 0; i < boids.Count; ++i)
        {
            for (int j = 0; j < enemies.Count; ++j)
            {
                float dist = (
                  enemies[j].transform.position -
                  boids[i].transform.position).magnitude;
                if (dist < sepDist)
                {
                    Vector3 targetDirection = (
                      boids[i].transform.position -
                      enemies[j].transform.position).normalized;

                    boids[i].TargetDirection += targetDirection;
                    boids[i].TargetDirection.Normalize();

                    boids[i].MaxSpeed *= sepWeight;
                }
            }
        }
    }

    [BurstCompile]
    struct FlockJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> Velocities;
        [WriteOnly] public NativeArray<float3> NewPositions;
        [WriteOnly] public NativeArray<float3> NewVelocities;

        [ReadOnly] public float3 BoundsMin;
        [ReadOnly] public float3 BoundsMax;
        [ReadOnly] public float Visibility;
        [ReadOnly] public float SeparationDistance;
        [ReadOnly] public float WeightAlignment;
        [ReadOnly] public float WeightSeparation;
        [ReadOnly] public float WeightCohesion;

        [ReadOnly] public float DeltaTime; // Pass deltaTime from the main thread

        public void Execute(int index)
        {
            // Get the current position and velocity of the boid
            float3 position = Positions[index];
            float3 velocity = Velocities[index];

            // Set the new position and velocity
            NewPositions[index] = position + velocity * DeltaTime;
            NewVelocities[index] = velocity;
        }
    }
}
