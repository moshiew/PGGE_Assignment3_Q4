using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UI : MonoBehaviour
{
  public Text textNumBoids;
  public Text textNumEnemies;

  public FlockBehaviour flockBehaviour;

  void Start()
  {
    StartCoroutine(Coroutine_UpdateText());
  }

  IEnumerator Coroutine_UpdateText()
  {
    while(true)
    {
      int enemyCount = 0;
      int boidCount = 0;
      foreach(Flock flock in flockBehaviour.flocks)
      {
        if (flock.isPredator)
          enemyCount += flock.mAutonomous.Count;
        else
          boidCount += flock.mAutonomous.Count;
      }
      textNumBoids.text = "Boids: " + boidCount.ToString();
      textNumEnemies.text = "Predators: " + enemyCount.ToString();
      yield return new WaitForSeconds(0.5f);
    }
  }
}
