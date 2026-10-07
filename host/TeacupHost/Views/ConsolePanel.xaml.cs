using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Threading;
using TeacupHost.ViewModels;

namespace TeacupHost.Views;

public partial class ConsolePanel : UserControl
{
    private MainViewModel? _vm;

    public ConsolePanel()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
    }

    private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
    {
        if (_vm != null)
            _vm.LogAppended -= OnLogAppended;
        _vm = DataContext as MainViewModel;
        if (_vm != null)
            _vm.LogAppended += OnLogAppended;
    }

    private void OnLogAppended()
    {
        if (_vm is not { AutoScrollConsole: true } || Log.Items.Count == 0)
            return;
        Dispatcher.BeginInvoke(() =>
        {
            if (Log.Items.Count > 0)
                Log.ScrollIntoView(Log.Items[^1]);
        }, DispatcherPriority.Background);
    }

    private void OnInputKeyDown(object sender, KeyEventArgs e)
    {
        if (_vm == null)
            return;
        switch (e.Key)
        {
            case Key.Enter:
                if (_vm.SendConsoleCommand.CanExecute(null))
                    _vm.SendConsoleCommand.Execute(null);
                e.Handled = true;
                break;
            case Key.Up:
                _vm.HistoryUp();
                Input.CaretIndex = Input.Text.Length;
                e.Handled = true;
                break;
            case Key.Down:
                _vm.HistoryDown();
                Input.CaretIndex = Input.Text.Length;
                e.Handled = true;
                break;
        }
    }
}
