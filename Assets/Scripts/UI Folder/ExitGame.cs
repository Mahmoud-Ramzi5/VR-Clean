using UnityEngine;
using UnityEngine.UI;

public class ExitGame : MonoBehaviour
{
    private Button exitButton;

    private void Start()
    {
        exitButton = GetComponent<Button>();
        exitButton.onClick.AddListener(QuitGame);
    }

    public void QuitGame()
    {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
    }
}