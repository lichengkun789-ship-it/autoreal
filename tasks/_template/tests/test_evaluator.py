from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def test_template_evaluator_is_explicitly_unverified():
    path = Path(__file__).resolve().parents[1] / "evaluator.py"
    spec = spec_from_file_location("task_template_evaluator", path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.evaluate({})["status"] == "unverified"
