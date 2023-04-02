from cmake_python_project.example import add_one

def test_add_one():
    assert add_one(1) == 2
    assert add_one(2) == 3