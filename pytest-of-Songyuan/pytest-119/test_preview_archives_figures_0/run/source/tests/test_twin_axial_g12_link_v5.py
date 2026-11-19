import errno
import inspect
import pytest
from ros2.twin_axial_g12_link_v5.ipc import JsonlReader


def test_partial_utf8_and_multiple_lines(tmp_path):
    p=tmp_path/'ipc';r=JsonlReader(p)
    assert r.read()==[]
    p.write_bytes(b'{"a":"\xe4')
    assert r.read()==[] and r.offset==0
    p.write_bytes('{"a":"中"}\n{"b":2}\n{"c":'.encode())
    assert r.read()==[{'a':'中'},{'b':2}]
    offset=r.offset
    assert r.read()==[] and r.offset==offset
    with p.open('ab') as f:f.write(b'3}\n')
    assert r.read()==[{'c':3}]


def test_enodata_no_commit_and_bounded_retry(tmp_path,monkeypatch):
    events=[];r=JsonlReader(tmp_path/'ipc',events.append);r.offset=17
    def fail():raise OSError(getattr(errno,'ENODATA',61),'No data available')
    monkeypatch.setattr(r,'_read',fail)
    with pytest.raises(OSError):r.read()
    assert r.offset==17 and len(events)==3


def test_enodata_then_recovery(tmp_path,monkeypatch):
    p=tmp_path/'ipc';p.write_text('{"n":1}\n');r=JsonlReader(p);original=r._read
    calls=[]
    def once():
        calls.append(1)
        if len(calls)==1:raise OSError(getattr(errno,'ENODATA',61),'No data available')
        return original()
    monkeypatch.setattr(r,'_read',once)
    assert r.read()==[{'n':1}] and r.read()==[]


@pytest.mark.parametrize('bad',[b'\n',b'[]\n',b'{oops}\n'])
def test_bad_complete_line_never_commits(tmp_path,bad):
    p=tmp_path/'ipc';p.write_bytes(b'{"ok":1}\n'+bad);r=JsonlReader(p)
    with pytest.raises(ValueError):r.read()
    assert r.offset==0


def test_truncation_and_disappearance(tmp_path):
    p=tmp_path/'ipc';p.write_text('{"n":1}\n');r=JsonlReader(p);r.read();p.write_text('')
    with pytest.raises(ValueError,match='truncated'):r.read()
    p.unlink()
    with pytest.raises(ValueError,match='disappeared'):r.read()


def test_watchdog_unchanged():
    from ros2.twin_axial_g12_real_v3.gateway import Gate as Old
    from ros2.twin_axial_g12_link_v5.gateway import Gate as New
    assert inspect.getsource(Old)==inspect.getsource(New)
    gate=New();gate.update({'X':1,'M':0},0,0,0)
    assert gate.applied(.26,.1)[2]
    assert gate.applied(.1,1.01)[2]
