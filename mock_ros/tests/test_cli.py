import sys
import pytest
from irobot.cli import main

def test_cli_no_args(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["iRobot"])
    with pytest.raises(SystemExit) as exc:
        main()
    assert exc.value.code == 1

    out, err = capsys.readouterr()
    assert "usage:" in out.lower()

def test_cli_invalid_module(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["iRobot", "run", "no_such_pkg", "no_such_node"])
    with pytest.raises(SystemExit) as exc:
        main()
    assert exc.value.code == 1

    out, err = capsys.readouterr()
    assert "could not import irobot.no_such_pkg.no_such_node" in err.lower()
