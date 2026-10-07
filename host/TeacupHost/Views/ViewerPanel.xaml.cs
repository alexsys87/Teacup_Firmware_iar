using System.Windows;
using System.Windows.Controls;

namespace TeacupHost.Views;

public partial class ViewerPanel : UserControl
{
    public ViewerPanel()
    {
        InitializeComponent();
    }

    private void OnIso(object sender, RoutedEventArgs e) => View3D.ViewIso();
    private void OnTop(object sender, RoutedEventArgs e) => View3D.ViewTop();
    private void OnFront(object sender, RoutedEventArgs e) => View3D.ViewFront();
    private void OnBed(object sender, RoutedEventArgs e) => View3D.ViewBed();
    private void OnReset(object sender, RoutedEventArgs e) => View3D.ResetView();
    private void OnFitModel(object sender, RoutedEventArgs e) => View2D.FitModel();
    private void OnFitBed(object sender, RoutedEventArgs e) => View2D.FitBed();
}
