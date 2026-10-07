using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using TeacupHost.ViewModels;

namespace TeacupHost.Views;

/// <summary>G-code listing: follows the current line; a click starts a simulation up to the clicked line.</summary>
public partial class GCodePanel : UserControl
{
    private MainViewModel? _vm;
    private bool _syncing;
    private bool _scrollPending;
    private ScrollViewer? _scroll;

    public GCodePanel()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
    }

    private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
    {
        if (_vm != null)
            _vm.PropertyChanged -= OnVmChanged;
        _vm = DataContext as MainViewModel;
        if (_vm != null)
            _vm.PropertyChanged += OnVmChanged;
    }

    private void OnVmChanged(object? sender, PropertyChangedEventArgs e)
    {
        if (e.PropertyName != nameof(MainViewModel.CurrentLine) || _scrollPending)
            return;
        // Coalesce: during a fast simulation the line changes on every frame.
        _scrollPending = true;
        Dispatcher.BeginInvoke(SyncSelection, DispatcherPriority.Background);
    }

    private void SyncSelection()
    {
        _scrollPending = false;
        if (_vm == null)
            return;
        int line = _vm.CurrentLine;
        _syncing = true;
        try
        {
            if (line < 0 || line >= Lines.Items.Count)
            {
                Lines.SelectedIndex = -1;
                return;
            }
            Lines.SelectedIndex = line;
            // Keep the current line in the middle (item based scrolling).
            _scroll ??= FindScrollViewer(Lines);
            if (_scroll != null)
                _scroll.ScrollToVerticalOffset(Math.Max(0, line - _scroll.ViewportHeight / 2 + 1));
            else
                Lines.ScrollIntoView(Lines.Items[line]);
        }
        finally
        {
            _syncing = false;
        }
    }

    private static ScrollViewer? FindScrollViewer(DependencyObject root)
    {
        for (int i = 0; i < VisualTreeHelper.GetChildrenCount(root); i++)
        {
            var child = VisualTreeHelper.GetChild(root, i);
            if (child is ScrollViewer sv)
                return sv;
            var found = FindScrollViewer(child);
            if (found != null)
                return found;
        }
        return null;
    }

    private void OnSelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (_syncing || _vm == null || Lines.SelectedIndex < 0)
            return;
        if (!Lines.IsKeyboardFocusWithin && !Lines.IsMouseOver)
            return;
        _vm.JumpToLine(Lines.SelectedIndex);
    }
}
