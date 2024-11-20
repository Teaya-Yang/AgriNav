Shader "Hidden/Blur2"

{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _BlurRadius ("Blur Radius", Range(0, 10)) = 2.0
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

            struct appdata_t
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            float _BlurRadius;
            sampler2D _MainTex;

            v2f vert (appdata_t v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            half4 frag (v2f i) : SV_Target
            {
                // Gaussian weights
                float weights[9] = {0.05, 0.09, 0.12, 0.15, 0.18, 0.15, 0.12, 0.09, 0.05};

                // Calculate texel size
                float2 texelSize = 1.0 / _ScreenParams.xy;

                // Sample the texture multiple times and accumulate the result
                half4 col = half4(0, 0, 0, 0);
                for (int x = -4; x <= 4; x++)
                {
                    for (int y = -4; y <= 4; y++)
                    {
                        col += tex2D(_MainTex, i.uv + float2(x, y) * _BlurRadius * texelSize) * weights[x + 4] * weights[y + 4];
                    }
                }
                
                return col;
            }
            ENDCG
        }
    }
}
