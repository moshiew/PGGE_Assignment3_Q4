using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Autonomous : MonoBehaviour
{
  public float MaxSpeed = 10.0f;

  public float Speed
  {
    get;
    private set;
  } = 0.0f;

  public Vector2 accel = new Vector2(0.0f, 0.0f);

  public float TargetSpeed = 0.0f;
  public Vector3 TargetDirection = Vector3.zero;
  public float RotationSpeed = 0.0f;

  public SpriteRenderer spriteRenderer;

  // Start is called before the first frame update
  void Start()
  {
    Speed = 0.0f;
    SetRandomSpeed();
    SetRandomDirection();
  }

  void SetRandomSpeed()
  {
    float speed = Random.Range(0.0f, MaxSpeed);
  }

  void SetRandomDirection()
  {
    float angle = 30.0f;// Random.Range(-180.0f, 180.0f);
    Vector2 dir = new Vector2(Mathf.Cos(Mathf.Deg2Rad * angle), Mathf.Sin(Mathf.Deg2Rad * angle));//, 0.0f);
    dir.Normalize();
    TargetDirection = dir;
  }

  public void SetColor(Color c)
  {
    spriteRenderer.color = c;
  }

  // Update is called once per frame
  public void Update()
  {
    Vector3 targetDirection = TargetDirection;
    targetDirection.Normalize();

    Vector3 rotatedVectorToTarget = 
      Quaternion.Euler(0, 0, 90) * 
      targetDirection;

    Quaternion targetRotation = Quaternion.LookRotation(
      forward: Vector3.forward,
      upwards: rotatedVectorToTarget);

    transform.rotation = Quaternion.RotateTowards(
      transform.rotation, 
      targetRotation, 
      RotationSpeed * Time.deltaTime);

    Speed = Speed + ((TargetSpeed - Speed)/10.0f) * Time.deltaTime;

    if (Speed > MaxSpeed)
      Speed = MaxSpeed;

    transform.Translate(Vector3.right * Speed * Time.deltaTime, Space.Self);
  }

  private void FixedUpdate()
  {
  }

  private IEnumerator Coroutine_LerpTargetSpeed(
    float start,
    float end,
    float seconds = 2.0f)
  {
    float elapsedTime = 0;
    while (elapsedTime < seconds)
    {
      Speed = Mathf.Lerp(
        start,
        end,
        (elapsedTime / seconds));
      elapsedTime += Time.deltaTime;

      yield return null;
    }
    Speed = end;
  }

  private IEnumerator Coroutine_LerpTargetSpeedCont(
  float seconds = 2.0f)
  {
    float elapsedTime = 0;
    while (elapsedTime < seconds)
    {
      Speed = Mathf.Lerp(
        Speed,
        TargetSpeed,
        (elapsedTime / seconds));
      elapsedTime += Time.deltaTime;

      yield return null;
    }
    Speed = TargetSpeed;
  }

  static public Vector3 GetRandom(Vector3 min, Vector3 max)
  {
    return new Vector3(
      Random.Range(min.x, max.x), 
      Random.Range(min.y, max.y), 
      Random.Range(min.z, max.z));
  }
}
