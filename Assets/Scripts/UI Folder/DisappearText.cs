using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class DisappearText : MonoBehaviour
{
    private TMP_Text textComponent;

    void Start()
    {
        textComponent = GetComponent<TMP_Text>();

        if (textComponent == null)
        {
            Debug.LogError("No Text component found on this GameObject!");
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            textComponent.enabled = false;
        }
    }
}