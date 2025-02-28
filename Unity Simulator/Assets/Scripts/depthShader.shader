Shader "Hidden/DepthEffects"
{
    Properties
    {
        _MainTex ("", 2D) = "white" {}
        _DepthScale ("Depth Scale", Float) = 0.001 // Adjust this to scale depth
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            uniform float _DepthScale;

            struct v2f
            {
                float4 pos : SV_POSITION;
                float depth : TEXCOORD0;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            // Convert non-linear depth to linear depth in meters
            float LinearDepth(float depth01)
            {
                float near = _ProjectionParams.y;
                float far = _ProjectionParams.z;
                return near * far / (far - depth01 * (far - near));
            }

            v2f vert(appdata_base v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.pos = UnityObjectToClipPos(v.vertex);
                o.depth = COMPUTE_DEPTH_01; // Gets non-linear depth
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                float depthInMeters = LinearDepth(i.depth);
                return depthInMeters * _DepthScale; // Store scaled depth in output
            }
            ENDCG
        }
    }

    Fallback Off
}

