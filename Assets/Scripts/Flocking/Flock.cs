using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Flock
{
  public GameObject PrefabBoid;

  [Space(10)]
  [Header("A flock is a group of Automous objects.")]
  public string name = "Default Name";
  public int numBoids = 10;
  public bool isPredator = false;
  public Color colour = new Color(1.0f, 1.0f, 1.0f);
  public float maxSpeed = 20.0f;
  public float maxRotationSpeed = 200.0f;
  [Space(10)]

  [Header("Flocking Rules")]
  public bool useRandomRule = true;
  public bool useAlignmentRule = true;
  public bool useCohesionRule = true;
  public bool useSeparationRule = true;
  public bool useFleeOnSightEnemyRule = true;
  public bool useAvoidObstaclesRule = true;
  [Space(10)]

  [Header("Rule Weights")]
  [Range(0.0f, 10.0f)]
  public float weightRandom = 1.0f;
  [Range(0.0f, 10.0f)]
  public float weightAlignment = 2.0f;
  [Range(0.0f, 10.0f)]
  public float weightCohesion = 3.0f;
  [Range(0.0f, 10.0f)]
  public float weightSeparation = 8.0f;
  [Range(0.0f, 50.0f)]
  public float weightFleeOnSightEnemy = 50.0f;
  [Range(0.0f, 50.0f)]
  public float weightAvoidObstacles = 10.0f;

  [Space(10)]
  [Header("Properties")]
  public float separationDistance = 5.0f;
  public float enemySeparationDistance = 5.0f;
  public float visibility = 20.0f;
  public bool bounceWall = true;

  [HideInInspector]
  public List<Autonomous> mAutonomous = new List<Autonomous>();

  public Flock()
  {
  }
}
