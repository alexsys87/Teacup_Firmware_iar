using System.Globalization;
using System.Windows;

namespace TeacupHost.Services;

/// <summary>
/// UI language: the strings live in Resources/Strings.{ru,en}.xaml. XAML
/// uses them with DynamicResource, code with <see cref="T"/>; switching the
/// language swaps the dictionary and raises <see cref="Changed"/>.
/// </summary>
public static class Loc
{
    public const string Russian = "ru";
    public const string English = "en";

    public static string Language { get; private set; } = Russian;

    public static event Action? Changed;

    public static CultureInfo Culture => CultureInfo.GetCultureInfo(Language == Russian ? "ru-RU" : "en-US");

    /// <summary>Language of the system, if it's one of ours, else English.</summary>
    public static string SystemLanguage =>
        CultureInfo.CurrentUICulture.TwoLetterISOLanguageName == Russian ? Russian : English;

    public static void Apply(string language)
    {
        language = language == Russian ? Russian : English;
        Language = language;
        ReplaceDictionary("Resources/Strings.", $"Resources/Strings.{language}.xaml");
        Changed?.Invoke();
    }

    /// <summary>Text of a string resource, the key itself if it's missing.</summary>
    public static string T(string key) =>
        Application.Current?.TryFindResource(key) as string ?? key;

    /// <summary>A string resource used as a format.</summary>
    public static string F(string key, params object[] args) => string.Format(Culture, T(key), args);

    /// <summary>Put a dictionary from this assembly in place of the merged one whose source contains <paramref name="marker"/>.</summary>
    internal static void ReplaceDictionary(string marker, string path)
    {
        var merged = Application.Current.Resources.MergedDictionaries;
        var dict = new ResourceDictionary
        {
            Source = new Uri($"pack://application:,,,/TeacupHost;component/{path}", UriKind.Absolute),
        };
        for (int i = 0; i < merged.Count; i++)
        {
            if (merged[i].Source?.OriginalString.Contains(marker, StringComparison.OrdinalIgnoreCase) == true)
            {
                merged[i] = dict;
                return;
            }
        }
        merged.Add(dict);
    }
}

/// <summary>Light or dark theme: WPF UI's dictionaries plus our colors (Resources/Theme.*.xaml).</summary>
public static class ThemeService
{
    public static bool IsDark { get; private set; } = true;

    public static event Action? Changed;

    public static void Apply(bool dark)
    {
        IsDark = dark;
        Loc.ReplaceDictionary("Resources/Theme.", dark ? "Resources/Theme.Dark.xaml" : "Resources/Theme.Light.xaml");
        try
        {
            Wpf.Ui.Appearance.ApplicationThemeManager.Apply(
                dark ? Wpf.Ui.Appearance.ApplicationTheme.Dark : Wpf.Ui.Appearance.ApplicationTheme.Light,
                Wpf.Ui.Controls.WindowBackdropType.Mica, true);
        }
        catch (Exception ex) when (ex is NotImplementedException or EntryPointNotFoundException or
                                       System.ComponentModel.Win32Exception)
        {
            // The control colors are switched before the window backdrop
            // (Mica) is updated; without DWM support only the backdrop stays.
        }
        Changed?.Invoke();
    }
}
