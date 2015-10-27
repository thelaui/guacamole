  //steppo
  //vec3 tmp_ms_v       = normalize( cross(ms_n,ms_u) );
  //VertexOut.pass_ms_u = normalize( cross(tmp_ms_v,ms_n) ) * radius_importance_scaling  * 2.0 * in_radius;
  //VertexOut.pass_ms_v = normalize(tmp_ms_v) * 2.0 * in_radius ;

  //adrian

float clamped_radius = in_radius;

if (in_radius > 0.1) {
  clamped_radius = 0.0;
}

VertexOut.pass_ms_u = normalize(ms_u) * radius_scaling * clamped_radius;
VertexOut.pass_ms_v = normalize(cross(ms_n, ms_u)) * radius_scaling  * clamped_radius;

