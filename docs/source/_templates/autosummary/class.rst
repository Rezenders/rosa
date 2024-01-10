{{ objname | escape | underline}}

.. currentmodule:: {{ module }}

.. autoclass:: {{ objname }}
  :members:
  :undoc-members:
  :special-members: __init__

  {% if methods %}
  .. rubric:: {{ _('Methods') }}

  .. autosummary::
  {% for item in methods %}
  {%- if item not in inherited_members %}
     ~{{ name }}.{{ item }}
  {%- endif %}
  {%- endfor %}
  {% endif %}

  {% if attributes %}
  .. rubric:: {{ _('Attributes') }}

  .. autosummary::
  {% for item in attributes %}
  {%- if item not in inherited_members %}
     ~{{ name }}.{{ item }}
  {%- endif %}
  {%- endfor %}
  {% endif %}
