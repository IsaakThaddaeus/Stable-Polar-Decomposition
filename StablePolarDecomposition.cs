using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class StablePolarDecomposition
{

    public static Vector3[] matchShape(Vector3[] verticesInitial, Vector3[] verticesDeformed, Vector3 centerOfMassInitial, Quaternion quat)
    {
        Vector3 centerOfMassSimulated = getCenterOfMass(verticesDeformed);
        Vector3[] matchedVertices = new Vector3[verticesInitial.Length];
        float[,] apq = new float[3, 3];


        for (int i = 0; i < verticesInitial.Length; i++)
        {
            Vector3 q = verticesInitial[i] - centerOfMassInitial;
            Vector3 p = verticesDeformed[i] - centerOfMassSimulated;

            apq[0, 0] += p.x * q.x;
            apq[0, 1] += p.x * q.y;
            apq[0, 2] += p.x * q.z;

            apq[1, 0] += p.y * q.x;
            apq[1, 1] += p.y * q.y;
            apq[1, 2] += p.y * q.z;

            apq[2, 0] += p.z * q.x;
            apq[2, 1] += p.z * q.y;
            apq[2, 2] += p.z * q.z;
        }

        Matrix4x4 A = new Matrix4x4();
        A.SetRow(0, new Vector4(apq[0, 0], apq[0, 1], apq[0, 2], 0));
        A.SetRow(1, new Vector4(apq[1, 0], apq[1, 1], apq[1, 2], 0));
        A.SetRow(2, new Vector4(apq[2, 0], apq[2, 1], apq[2, 2], 0));

        quat = ExtractRotation(A, quat, 10);

        for (int i = 0; i < verticesInitial.Length; i++)
        {
            Vector3 q = verticesInitial[i] - centerOfMassInitial;
            Vector3 p = quat * q;

            matchedVertices[i] = p + centerOfMassSimulated;
        }

        return matchedVertices;
    }
    private static Quaternion ExtractRotation(Matrix4x4 A, Quaternion q, int maxIter)
    {
        for (int iter = 0; iter < maxIter; iter++)
        {
            Matrix4x4 R = Matrix4x4.Rotate(q);

            Vector3 omega = Vector3.Cross(R.GetColumn(0), A.GetColumn(0)) +
                            Vector3.Cross(R.GetColumn(1), A.GetColumn(1)) +
                            Vector3.Cross(R.GetColumn(2), A.GetColumn(2));

            float dotProduct = Mathf.Abs(Vector3.Dot(R.GetColumn(0), A.GetColumn(0))) +
                               Mathf.Abs(Vector3.Dot(R.GetColumn(1), A.GetColumn(1))) +
                               Mathf.Abs(Vector3.Dot(R.GetColumn(2), A.GetColumn(2)));

            float epsilon = 1.0e-9f;
            omega *= (1.0f / (dotProduct + epsilon));
            float w = omega.magnitude;

            if (w < 1.0e-9f)
                break;
            

            q = Quaternion.AngleAxis(w * Mathf.Rad2Deg, omega.normalized) * q;
            q.Normalize();
        }

        return q;
    }
    public static Vector3 getCenterOfMass(Vector3[] vertices)
    {
        Vector3 center = Vector3.zero;
        float totalMass = 0;

        for (int i = 0; i < vertices.Length; i++)
        {
            center += vertices[i] * 1;
            totalMass += 1;
        }

        return center / totalMass;
    }

}
