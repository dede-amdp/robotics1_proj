void speed_estimation(ringbuffer_t *q_actual, ringbuffer_t *dq_actual, float reduction, float *v_est, float *a_est){
    float now, esp;
    float A[ESTIMATION_STEPS*ESTIMATION_STEPS], X[ESTIMATION_STEPS], P[ESTIMATION_STEPS], invA[ESTIMATION_STEPS*ESTIMATION_STEPS];
    /* temp matrices */
    float trM[ESTIMATION_STEPS*ESTIMATION_STEPS], tempM[ESTIMATION_STEPS*ESTIMATION_STEPS];
    float adjM[ESTIMATION_STEPS*ESTIMATION_STEPS], subM[(ESTIMATION_STEPS-1)*(ESTIMATION_STEPS-1)];
    float invM[ESTIMATION_STEPS*ESTIMATION_STEPS], dotM[ESTIMATION_STEPS*ESTIMATION_STEPS];


    if(q_actual->length < 10){
        // rblast(q_actual,&X[0]); /* get newest value */
        float a[4];
        rbget(q_actual, 0, &a[0]);
        rbget(q_actual, 1, &a[1]);
        rbget(dq_actual, 0, &a[2]);
        rbget(dq_actual, 1, &a[3]);
        /*uint8_t index1 = (q_actual->head) + (q_actual->length) - 1;
        uint8_t index2 = (dq_actual->head) + (dq_actual->length) - 1;
        a[0] = q_actual->buffer[index1%RBUF_SZ];
        a[1] = q_actual->buffer[(index1-1+RBUF_SZ)%RBUF_SZ];
        a[2] = dq_actual->buffer[index2%RBUF_SZ];
        a[3] = dq_actual->buffer[(index2-1+RBUF_SZ)%RBUF_SZ];
        /* if not enough data is available, apply simple estimation */
        *v_est = ((a[1]-a[0])/T_S);
        *a_est = (a[3]-a[2])/T_S;
        // disp1 = *v_est;
        return;
    }else{
    	// SECTION DEBUG
    	// rblast(q_actual,&X[0]); // get newest value 
        float a[4];
		rbget(q_actual, 0, &a[0]);
		rbget(q_actual, 1, &a[1]);
		rbget(dq_actual, 0, &a[2]);
		rbget(dq_actual, 1, &a[3]);
		uint8_t index1 = (q_actual->head) + (q_actual->length) - 1;
		uint8_t index2 = (dq_actual->head) + (dq_actual->length) - 1;
		a[0] = q_actual->buffer[index1%RBUF_SZ];
		a[1] = q_actual->buffer[(index1-1+RBUF_SZ)%RBUF_SZ];
		a[2] = dq_actual->buffer[index2%RBUF_SZ];
		a[3] = dq_actual->buffer[(index2-1+RBUF_SZ)%RBUF_SZ];
		// if not enough data is available, apply simple estimation
		*v_est = ((a[1]-a[0])/T_S);
		*a_est = (a[3]-a[2])/T_S;
		// disp1 = *v_est;
		// disp2 = *a_est;
		return;
		// !SECTION DEBUG
    }

    now = (float) HAL_GetTick()/1000; /* time passed from when the process launch */
    uint8_t i,j;
    for(i = 0; i < ESTIMATION_STEPS; i++){
        for(j = 0; j < ESTIMATION_STEPS; j++){
            A[j+i*ESTIMATION_STEPS] = pow((float)(now - i*T_S), (float) ESTIMATION_STEPS-j-1);
        }
    }

    for(i = 0; i < ESTIMATION_STEPS; i++){
        rbget(q_actual, i, &X[ESTIMATION_STEPS-1-i]);
    }
    /*
        x(t) = sum(p_i*t^i)
        v(t) = sum(i*p_i*t^(i-1))
        a(t) = sum(i*(i-1)*p_i*t^(i-2))
        ---
        p_i -> P[i]
        x_i=A_i*P -> X = [x_0; x_1; ...; x_n] = [A_0; A_1; ...; A_n]*P = A*P -> P = A^(-1)*X = (A^T*A)^(-1)*A^T*X
    */

    pseudo_inv(A, trM, tempM, adjM, subM, invM, dotM, ESTIMATION_STEPS, invA);
    dot(invA, ESTIMATION_STEPS, ESTIMATION_STEPS, X, ESTIMATION_STEPS, 1, P);
    *v_est = 0;
    *a_est = 0;
    for(i = 0; i < ESTIMATION_STEPS; i++){
        esp = (ESTIMATION_STEPS-i-1);
        /* the derivation of constant values is 0 -> exclude the derivative of the constant values from the computation otherwise it would be now^i with i < 0 */
        if(esp-1 >= 0){
            *v_est += esp*pow(now, esp-1)*P[i];
        }
        if(esp-2 >= 0){ 
            *a_est += esp*(esp-1)*pow(now, esp-2)*P[i];
        }
    }
}