package org.team5987.annotation.command_enum

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.ClassKind
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.squareup.kotlinpoet.*
import com.squareup.kotlinpoet.ksp.writeTo

const val ANNOTATION_PACKAGE =
    "org.team5987.annotation.command_enum.CommandEnum"

val snakeRegex = "_[a-zA-Z]".toRegex()

fun String.snakeToCamelCase(): String {
    return snakeRegex.replace(lowercase()) {
        it.value.replace("_", "").uppercase()
    }
}

class CreateCommandProcessor(env: SymbolProcessorEnvironment) :
    SymbolProcessor {
    private val code = env.codeGenerator

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols = resolver.getSymbolsWithAnnotation(ANNOTATION_PACKAGE)

        symbols
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_CLASS }
            .forEach { enumDecl ->
                generateForEnum(enumDecl)
            }

        return emptyList()
    }

    fun generateInterface(
        pkg: String,
        fileName: String,
        enumName: String,
        entries: List<String>,
        priorityPropertyName: String?,
    ): FileSpec {
        val enumClass = ClassName(pkg, enumName)
        val commandClass = ClassName("org.wpilib.command3", "Command")
        val unnamedCommandClass =
            ClassName("frc.robot.lib.commands", "UnnamedCommand")
        val optInClass = ClassName("kotlin", "OptIn")
        val setTargetOptInMarkerClass =
            ClassName(
                "org.team5987.annotation.command_enum",
                "CommandEnumSetTargetOptIn",
            )

        val entryFunctions = entries.map { entry ->
            val camelEntry = entry.snakeToCamelCase()
            val funBuilder = FunSpec.builder(camelEntry).returns(commandClass)
            funBuilder.addAnnotation(
                AnnotationSpec.builder(optInClass)
                    .addMember("%T::class", setTargetOptInMarkerClass)
                    .build()
            )

            if (priorityPropertyName != null) {
                funBuilder.addStatement(
                    "return setTarget(%1T.%2L).withPriority(%1T.%2L.%4L.priority).named(%3S)",
                    enumClass,
                    entry,
                    "${pkg.substringAfterLast(".")}/$camelEntry",
                    priorityPropertyName,
                )
            } else {
                funBuilder.addStatement(
                    "return setTarget(%1T.%2L).withPriority(%3T.DEFAULT_PRIORITY).named(%4S)",
                    enumClass,
                    entry,
                    commandClass,
                    "${pkg.substringAfterLast(".")}/$camelEntry",
                )
            }

            funBuilder.build()
        }

        val setTargetFun =
            FunSpec.builder("setTarget")
                .addParameter("value", enumClass)
                .returns(unnamedCommandClass)
                .addAnnotation(setTargetOptInMarkerClass)
                .addModifiers(KModifier.ABSTRACT)
                .build()

        val interfaceSpec =
            TypeSpec.interfaceBuilder(fileName)
                .addFunctions(entryFunctions)
                .addFunction(setTargetFun)
                .build()

        return FileSpec.builder(pkg, fileName).addType(interfaceSpec).build()
    }

    private fun generateForEnum(enumDecl: KSClassDeclaration) {
        val pkg = enumDecl.packageName.asString()
        val enumName = enumDecl.simpleName.asString()

        val entries =
            enumDecl.declarations
                .filterIsInstance<KSClassDeclaration>()
                .filter { it.classKind == ClassKind.ENUM_ENTRY }
                .map { it.simpleName.asString() }
                .toList()

        val priorityPropertyName: String? =
            enumDecl.primaryConstructor
                ?.parameters
                ?.firstOrNull { param ->
                    val paramType = param.type.resolve()
                    paramType.declaration.simpleName.asString() == "Priority" &&
                        paramType.declaration.packageName.asString() ==
                            "org.team5987.annotation.command_enum" &&
                        (param.isVal || param.isVar)
                }
                ?.name
                ?.asString()

        val fileName = "${enumName}CommandFactory"

        val generated: FileSpec =
            generateInterface(
                pkg,
                fileName,
                enumName,
                entries,
                priorityPropertyName,
            )

        generated.writeTo(code, Dependencies(false))
    }
}

class CommandEnumProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor {
        return CreateCommandProcessor(environment)
    }
}
