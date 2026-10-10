import xml.etree.ElementTree as ET

from configure_division_b import configure


def test_configuration_preserves_other_settings_and_is_idempotent(tmp_path):
    path = tmp_path / "grsim.xml"
    path.write_text('<VarXML><Var name="Network" type="list" /></VarXML>')
    assert configure(path, check=True)
    assert ET.parse(path).find("Var[@name='Geometry']") is None
    assert configure(path)
    assert not configure(path, check=True)
    assert not configure(path)
    root = ET.parse(path)
    assert root.find("Var[@name='Network']") is not None
    field = root.find("Var[@name='Geometry']/Var[@name='Field']/Var[@name='Division B']")
    assert float(field.find("Var[@name='Length']").text) == 9
    assert float(field.find("Var[@name='Width']").text) == 6
    assert float(field.find("Var[@name='Margin along the touch lines']").text) == 0.3
    assert path.with_suffix('.xml.before-division-b').exists()
