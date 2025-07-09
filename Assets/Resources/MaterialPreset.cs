using UnityEngine;

// [CreateAssetMenu(fileName = "MaterialPreset", menuName = "Materials/Material Preset")]
public class MaterialPreset : ScriptableObject
{
    [SerializeField, HideInInspector]
    private MaterialType type;

    // Unity
    public MaterialType Type => type;
    public Material material;
    public bool isRigid;
    public float totalMass;
    public float bounciness;
    public float friction;
    [Header("Spring Settings")]
    [Header("Spring Settings / Layer 1")]
    public float springConstantL1 = 100f;
    public float damperConstantL1 = 0.6f;
    public float connectionRadiusL1 = 1f;
    public float maxRestLengthL1 = 2f;
    [Header("Spring Settings / Layer 2")]
    public float springConstantL2 = 60f;
    public float damperConstantL2 = 0.5f;
    public float connectionRadiusL2 = 2f;
    public float maxRestLengthL2 = 2.5f;
    [Header("Spring Settings / Layer 3")]
    public float springConstantL3 = 40f;
    public float damperConstantL3 = 0.4f;
    public float connectionRadiusL3 = 3f;
    public float maxRestLengthL3 = 3f;

    public void SetType(MaterialType newType)
    {
        type = newType;
    }

    public MaterialPreset(MaterialType t,
        Material m, float tm, bool kkk, float b, float f,
        float scl1, float dcl1, float cr1, float mrl1,
        float scl2, float dcl2, float cr2, float mrl2,
        float scl3, float dcl3, float cr3, float mrl3)
    {
        type = t;
        isRigid = kkk;
        material = m;
        totalMass = tm;
        bounciness = b;
        friction = f;

        springConstantL1 = scl1;
        damperConstantL1 = dcl1;
        connectionRadiusL1 = cr1;
        maxRestLengthL1 = mrl1;
        springConstantL2 = scl2;
        damperConstantL2 = dcl2;
        connectionRadiusL2 = cr2;
        maxRestLengthL2 = mrl2;
        springConstantL3 = scl3;
        damperConstantL3 = dcl3;
        connectionRadiusL3 = cr3;
        maxRestLengthL3 = mrl3;
    }

    public MaterialPreset(bool kkk, float tm, float b, float f,
        float scl1, float dcl1, float cr1, float mrl1,
        float scl2, float dcl2, float cr2, float mrl2,
        float scl3, float dcl3, float cr3, float mrl3)
    {
        isRigid = kkk;
        totalMass = tm;
        bounciness = b;
        friction = f;

        springConstantL1 = scl1;
        damperConstantL1 = dcl1;
        connectionRadiusL1 = cr1;
        maxRestLengthL1 = mrl1;
        springConstantL2 = scl2;
        damperConstantL2 = dcl2;
        connectionRadiusL2 = cr2;
        maxRestLengthL2 = mrl2;
        springConstantL3 = scl3;
        damperConstantL3 = dcl3;
        connectionRadiusL3 = cr3;
        maxRestLengthL3 = mrl3;
    }

    public MaterialPreset()
    {
        type = MaterialType.Custom;
        isRigid = false;

        totalMass = 100f;
        bounciness = 0.5f;
        friction = 0.0f;

        springConstantL1 = 100f;
        damperConstantL1 = 0.6f;
        connectionRadiusL1 = 1f;
        maxRestLengthL1 = 2f;
        springConstantL2 = 60f;
        damperConstantL2 = 0.5f;
        connectionRadiusL2 = 2f;
        maxRestLengthL2 = 2.5f;
        springConstantL3 = 40f;
        damperConstantL3 = 0.4f;
        connectionRadiusL3 = 3f;
        maxRestLengthL3 = 3f;
    }
}