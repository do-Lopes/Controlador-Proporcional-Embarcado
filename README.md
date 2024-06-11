# Controlador proporcional
Projeto desenvolvido durante a disciplina de microcontrolados onde foi criado um controlador proporcional de um ventilador no kit de desenvolvido PIC18F4550.
## Sobre o projeto
Nesse projeto o usuário pode aumentar ou diminuir o valor da temperatura de referência a partir dos botões INT1 e INT2, respectivamente, além disso, o usuário pode aumentar ou diminuir a força da fonte de calor a partir do potenciometro ligado a RA3. A partir disso o sistema calcula a diferença de temperatura e altera o sinal PWM do ventilador ligado a RC2. 
## Tecnologias utilizadas
- C
- PIC18F4550

## Pré-requisitos
- MPLAB X IDE (v5.20)
- Compilador XC8 (v1.45)

## Como utilizar
- **Clonar o repositório:**
```bash
git clone https://github.com/do-Lopes/Controlador-Proporcional-Embarcado
```
- **No MPLAB X IDE:**
Abrir a raiz do projeto **Controlador-Proporcional-Embarcado-main**
- **Configurar compilador:**
Após abrir o projeto no MPLAB clicar com o botão direito na raiz do projeto e selecionar a opção **Properties** e alterar o campo **Compiler Toolchains** para o compilador XC8 (v1.45)

